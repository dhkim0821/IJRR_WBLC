#include "Mercury_StateEstimator.hpp"
#include "Mercury_StateProvider.hpp"
#include "Mercury_interface.hpp"
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Model.hpp>
#include "Mercury_DynaControl_Definition.h"
#include <Utils/pseudo_inverse.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>


// Mocap based Estimator
#include <Mercury_Controller/StateEstimator/BodyFootPosEstimator.hpp>

// Orientation Estimators
#include <Mercury_Controller/StateEstimator/BasicAccumulation.hpp>

// Velocity Estimators
#include <Mercury_Controller/StateEstimator/SimpleAverageEstimator.hpp>
#include "MoCapManager.hpp"

// TODO: body_ang_vel_ is defined in global frame currently although the truth is 
// rotation axis representing the floating base orientation changes
// as configuration change --> body (local) frame
Mercury_StateEstimator::Mercury_StateEstimator(RobotSystem* robot):
    base_cond_(0),
    b_using_jpos_(false),
    curr_config_(mercury::num_q),
    curr_qdot_(mercury::num_qdot)
{
    sp_ = Mercury_StateProvider::getStateProvider();
    robot_sys_ = robot;

    body_foot_est_ = new BodyFootPosEstimator(robot);
    ori_est_ = new BasicAccumulation();
    vel_est_ = new SimpleAverageEstimator();
    mocap_vel_est_ = new SimpleAverageEstimator();
}

Mercury_StateEstimator::~Mercury_StateEstimator(){
    delete body_foot_est_;
    delete vel_est_;
    delete mocap_vel_est_;
}

void Mercury_StateEstimator::_RBDL_TEST(){
    // TEST
    dynacore::Vector q(mercury::num_q); q.setZero();
    dynacore::Vector qdot(mercury::num_qdot); qdot.setZero();

    q[mercury_joint::virtual_Rw] = 1.0;
    
    q[mercury_joint::rightAbduction] = -0.2;
    q[mercury_joint::rightHip] = -0.2;
    q[mercury_joint::rightKnee] = 0.4;
 
    q[mercury_joint::leftAbduction] = -0.2;
    q[mercury_joint::leftHip] = -0.2;
    q[mercury_joint::leftKnee] = 0.4;
    
    // Orientation
    dynacore::Vect3 rpy; rpy.setZero();
    dynacore::Quaternion quat_floating;
    //rpy[1] = M_PI/4.;
    rpy[1] = M_PI/10.;
    dynacore::convert(rpy, quat_floating);

    q[mercury_joint::virtual_Rx] = quat_floating.x();
    q[mercury_joint::virtual_Ry] = quat_floating.y();
    q[mercury_joint::virtual_Rz] = quat_floating.z();
    q[mercury_joint::virtual_Rw] = quat_floating.w();
    dynacore::pretty_print(q, std::cout, "q");
    dynacore::pretty_print(qdot, std::cout, "qdot");

    robot_sys_->UpdateSystem(q, qdot);
    //_BasicTest();
    _ProjectionTest();
    exit(0);
}
void Mercury_StateEstimator::_pseudoInv(const dynacore::Matrix & J, 
        dynacore::Matrix & J_pinv){

    double threshold(0.00000000001);
    dynacore::Matrix A, Ainv;
    robot_sys_->getMassInertia(A);
    robot_sys_->getInverseMassInertia(Ainv);

    //dynacore::pseudoInverse(J, threshold, J_pinv);

    //A(8,8) *= 100.0;
    //A(11,11) *= 100.0;
    dynacore::Matrix A_einv = A.inverse();
    Ainv = A_einv;
    dynacore::pretty_print(A, std::cout, "A");
    dynacore::pretty_print(Ainv, std::cout, "Ainv");

    //dynacore::pretty_print(A_einv,std::cout, "A inv test");
    //dynacore::Matrix test = A*Ainv;
    //dynacore::pretty_print(test,std::cout, "test");

    Ainv.setIdentity();
    //Ainv.block(0,0, 6,6) *= 0.01;
    dynacore::Matrix lambda_inv = J * Ainv * J.transpose();
    dynacore::Matrix lambda;
    dynacore::pseudoInverse(lambda_inv, threshold, lambda);
    J_pinv = Ainv * J.transpose() * lambda;
}
void Mercury_StateEstimator::_ProjectionTest(){

    dynacore::Matrix Jbody, Jlfoot, Jrfoot, Jtmp;
    robot_sys_->getFullJacobian(mercury_link::body, Jtmp);
    Jbody = dynacore::Matrix::Zero(3,mercury::num_qdot);
    Jbody(0, 2) = 1.;
    Jbody.block(1,0, 2, mercury::num_qdot) = Jtmp.block(0,0, 2, mercury::num_qdot);
    robot_sys_->getFullJacobian(mercury_link::leftFoot, Jtmp);
    Jlfoot = Jtmp.block(3,0, 3, mercury::num_qdot);
    robot_sys_->getFullJacobian(mercury_link::rightFoot, Jtmp);
    Jrfoot = Jtmp.block(3,0, 3, mercury::num_qdot);

    dynacore::pretty_print(Jbody, std::cout, "Jbody");
    // 
    Jbody -= Jlfoot;
    //Jrfoot -= Jlfoot;
    Jrfoot.block(0,0, 3, mercury::num_virtual).setZero();

    dynacore::Matrix I_mtx(mercury::num_qdot, mercury::num_qdot);
    I_mtx.setIdentity();

    dynacore::Matrix Jbody_inv, Jlfoot_inv;
    _pseudoInv(Jbody, Jbody_inv);
    _pseudoInv(Jlfoot, Jlfoot_inv);

    dynacore::pretty_print(Jlfoot_inv, std::cout, "Jlfoot inv");
    dynacore::Matrix Nbody = I_mtx - Jbody_inv * Jbody;
    dynacore::Matrix Nlfoot = I_mtx - Jlfoot_inv * Jlfoot;

    dynacore::Matrix nullspace_check_left = Jlfoot_inv * Jlfoot;
    dynacore::pretty_print(nullspace_check_left, std::cout, "null check lfoot");
    dynacore::Matrix JcBody = Jbody * Nlfoot;
    dynacore::Matrix JcBody_inv;
    _pseudoInv(JcBody, JcBody_inv);
    dynacore::Matrix NcBody = I_mtx - JcBody_inv * JcBody;


    dynacore::Matrix JfootBody = Jrfoot * NcBody;

    dynacore::Matrix A, Ainv;
    robot_sys_->getInverseMassInertia(Ainv);
    robot_sys_->getMassInertia(A);
    dynacore::Matrix M_check = A;
    Eigen::JacobiSVD<dynacore::Matrix> svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    dynacore::pretty_print(svd.singularValues(), std::cout, "svd singular value");


    dynacore::Matrix lambda_inv = JcBody * Ainv * JcBody.transpose();
    dynacore::Matrix lambda;
    dynacore::pseudoInverse(lambda_inv, 0.00001, lambda);
    dynacore::pretty_print(lambda, std::cout, "lambda");
    //dynacore::pretty_print(Jbody, std::cout, "J body");
    //dynacore::pretty_print(Nbody, std::cout, "N body");

    //Nlfoot *= 100000.;
    dynacore::pretty_print(Jlfoot, std::cout, "J lfoot");
    dynacore::pretty_print(Jrfoot, std::cout, "J rfoot");
    dynacore::pretty_print(Nlfoot, std::cout, "Nlfoot");
    dynacore::pretty_print(JcBody, std::cout, "Jc body");
    dynacore::pretty_print(NcBody, std::cout, "Nc body");

    dynacore::pretty_print(JfootBody, std::cout, "Jfootbody");

}
void Mercury_StateEstimator::_BasicTest(){
    dynacore::Matrix J;
    dynacore::Vector JdotQdot;
    dynacore::Vect3 pos, vel, ang_vel;
    dynacore::Quaternion link_ori_quat;
    int link_idx = mercury_link::body;
    //int link_idx = mercury_link::rightFoot;

    robot_sys_->getFullJacobian(link_idx, J);
    robot_sys_->getFullJDotQdot(link_idx, JdotQdot);
    robot_sys_->getOri(link_idx, link_ori_quat);
    robot_sys_->getPos(link_idx, pos);
    robot_sys_->getLinearVel(link_idx, vel);
    robot_sys_->getAngularVel(link_idx, ang_vel);
    
    Eigen::Matrix3d ori_rot(link_ori_quat); 
    dynacore::Matrix ori_rot_mt(6,6); ori_rot_mt.setZero();
    ori_rot_mt.topLeftCorner(3,3) = ori_rot;
    ori_rot_mt.bottomRightCorner(3,3) = ori_rot;

    dynacore::Matrix J_rot = ori_rot_mt.transpose() * J;

    dynacore::pretty_print(J, std::cout, "Jacobian");
    dynacore::pretty_print(J_rot, std::cout, "Jacobian Local");
    dynacore::pretty_print(link_ori_quat, std::cout, "ori");
    dynacore::pretty_print(JdotQdot, std::cout, "JdotQdot");
    dynacore::pretty_print(pos, std::cout, "pos");
    dynacore::pretty_print(vel, std::cout, "vel");
    dynacore::pretty_print(ang_vel, std::cout, "ang_vel");
}
void Mercury_StateEstimator::Initialization(Mercury_SensorData* data){
    //_RBDL_TEST();
    _JointUpdate(data);
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->EstimatorInitialization(body_ori_, imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(body_ori_, body_ang_vel_);
    body_foot_est_->Initialization(body_ori_);

    _ConfigurationAndModelUpdate();
    
    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);
    ((BasicAccumulation*)ori_est_)->CoMStateInitialization(sp_->CoM_pos_, sp_->CoM_vel_);
    vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);

    _FootContactUpdate(data);
    sp_->SaveCurrentData(data, robot_sys_);
}

void Mercury_StateEstimator::Update(Mercury_SensorData* data){
    _JointUpdate(data);
    
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    std::vector<double> imu_inc(3);

    for(int i(0); i<3; ++i){
        imu_inc[i] = data->imu_inc[i];
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->setSensorData(imu_acc, imu_inc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori_, body_ang_vel_);

    static bool visit_once(false);
    if ((sp_->phase_copy_ == 2) && (!visit_once)){
        vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        body_foot_est_->Initialization(body_ori_);
        visit_once = true;
    }

    _ConfigurationAndModelUpdate();

    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);

    // CoM velocity data smoothing filter
    vel_est_->Update(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    vel_est_->Output(sp_->est_CoM_vel_[0], sp_->est_CoM_vel_[1]);

    // Mocap based body velocity estimator
    dynacore::Vect3 mocap_body_vel;
    body_foot_est_->Update();
    body_foot_est_->getMoCapBodyVel(mocap_body_vel);
    body_foot_est_->getMoCapBodyPos(body_ori_, sp_->est_mocap_body_pos_);
    mocap_vel_est_->Update(mocap_body_vel[0], mocap_body_vel[1]);
    mocap_vel_est_->Output(
            sp_->est_mocap_body_vel_[0], 
            sp_->est_mocap_body_vel_[1]);

    _FootContactUpdate(data);
    sp_->SaveCurrentData(data, robot_sys_);
}

void Mercury_StateEstimator::_JointUpdate(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;

    sp_->jjpos_config_.setZero();
    sp_->jjvel_qdot_.setZero();
    sp_->jjpos_config_[mercury::num_qdot] = 1.;

    for (int i(0); i<mercury::num_act_joint; ++i){
        if(b_using_jpos_){
            curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];    
        } else{
            curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        }
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];
        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];

        // Joint encoder update
        sp_->jjpos_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        sp_->jjvel_qdot_[mercury::num_virtual + i] = data->joint_jvel[i];
        sp_->mjpos_[i] = data->motor_jpos[i];
    }
}

void Mercury_StateEstimator::_ConfigurationAndModelUpdate(){
    // Local Frame Setting
    if(base_cond_ == base_condition::floating){
        curr_config_[3] = body_ori_.x();
        curr_config_[4] = body_ori_.y();
        curr_config_[5] = body_ori_.z();
        curr_config_[mercury::num_qdot] = body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = body_ang_vel_[i];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        dynacore::Vect3 foot_pos, foot_vel;
        robot_sys_->getPos(sp_->stance_foot_, foot_pos);
        robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
        curr_config_[0] = -foot_pos[0];
        curr_config_[1] = -foot_pos[1];
        curr_config_[2] = -foot_pos[2];
        curr_qdot_[0] = -foot_vel[0];
        curr_qdot_[1] = -foot_vel[1];
        curr_qdot_[2] = -foot_vel[2];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        /// Jpos based model update  ////////////////////////////////
        if(b_jpos_model_update_){
            sp_->jjpos_config_[3] = body_ori_.x();
            sp_->jjpos_config_[4] = body_ori_.y();
            sp_->jjpos_config_[5] = body_ori_.z();
            sp_->jjpos_config_[mercury::num_qdot] = body_ori_.w();
            for(int i(0); i<3; ++i)
                sp_->jjvel_qdot_[i+3] = body_ang_vel_[i];

            sp_->jjpos_robot_sys_->UpdateSystem(sp_->jjpos_config_, sp_->jjvel_qdot_);

            sp_->jjpos_robot_sys_->getPos(sp_->stance_foot_, foot_pos);
            sp_->jjpos_robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
            sp_->jjpos_config_[0] = -foot_pos[0];
            sp_->jjpos_config_[1] = -foot_pos[1];
            sp_->jjpos_config_[2] = -foot_pos[2];
            sp_->jjvel_qdot_[0] = -foot_vel[0];
            sp_->jjvel_qdot_[1] = -foot_vel[1];
            sp_->jjvel_qdot_[2] = -foot_vel[2];

            sp_->jjpos_robot_sys_->UpdateSystem(sp_->jjpos_config_, sp_->jjvel_qdot_);
        }
        /// END of Jpos based model update ///////////////////////////
    } else if (base_cond_ == base_condition::fixed){
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    } else if (base_cond_ == base_condition::lying){
        // pitch rotation (PI/2)
        curr_config_[4] = sin(M_PI/2.0/2.0);
        curr_config_[mercury::num_qdot] = cos(M_PI/2.0/2.0);

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    } else {
        printf("[Error] Incorrect base condition setup\n");
        exit(0);
    }
    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;
}

void Mercury_StateEstimator::_FootContactUpdate(Mercury_SensorData* data){
    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;
}

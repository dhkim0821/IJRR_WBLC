#include "DracoBip_StateEstimator.hpp"
#include "DracoBip_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

#include <DracoBip_Controller/StateEstimator/BasicAccumulation.hpp>
#include <DracoBip_Controller/StateEstimator/BodyEstimator.hpp>
#include <Filter/filters.hpp>

DracoBip_StateEstimator::DracoBip_StateEstimator(RobotSystem* robot):
    curr_config_(dracobip::num_q),
    curr_qdot_(dracobip::num_qdot)
{
    sp_ = DracoBip_StateProvider::getStateProvider();
    robot_sys_ = robot;
    ori_est_ = new BasicAccumulation();
    
    mocap_x_vel_est_ = new AverageFilter(dracobip::servo_rate, 0.01, 1.0);
    mocap_y_vel_est_ = new AverageFilter(dracobip::servo_rate, 0.01, 1.5);
    body_est_ = new BodyEstimator(robot);
}

DracoBip_StateEstimator::~DracoBip_StateEstimator(){
    delete ori_est_;
    delete body_est_;
    delete mocap_x_vel_est_;
    delete mocap_y_vel_est_;
}

void DracoBip_StateEstimator::Initialization(DracoBip_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[dracobip::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<dracobip::num_act_joint; ++i){
        curr_config_[dracobip::num_virtual + i] = data->jpos[i];
        curr_qdot_[dracobip::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->EstimatorInitialization(imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(body_ori, body_ang_vel);
    body_est_->Initialization(body_ori);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[dracobip::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];

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

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    //_RBDL_TEST();
    sp_->SaveCurrentData(robot_sys_);
    dynacore::Vect3 com_pos, com_vel;
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);
}

void DracoBip_StateEstimator::Update(DracoBip_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[dracobip::num_qdot] = 1.;

    for (int i(0); i<dracobip::num_act_joint; ++i){
        curr_config_[dracobip::num_virtual + i] = data->jpos[i];
        curr_qdot_[dracobip::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
 
    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
   
    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->setSensorData( imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[dracobip::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];
    
    robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    // Foot position based offset
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

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    //dynacore::pretty_print(sp_->Q_, std::cout, "state estimator config");

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;
    
    sp_->SaveCurrentData(robot_sys_);

    // Mocap based body velocity 
    dynacore::Vect3 mocap_body_vel;
    body_est_->Update();
    body_est_->getMoCapBodyVel(mocap_body_vel);
    mocap_x_vel_est_->input(mocap_body_vel[0]);
    mocap_y_vel_est_->input(mocap_body_vel[1]);

    sp_->est_mocap_body_vel_[0] = mocap_x_vel_est_->output();
    sp_->est_mocap_body_vel_[1] = mocap_y_vel_est_->output();
}


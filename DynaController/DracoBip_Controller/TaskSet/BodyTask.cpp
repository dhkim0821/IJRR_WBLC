#include "BodyTask.hpp"
// Task consist of virtual joint (6) 
// (X, Y, Z), (Rx, Ry, Rz)

#include <Configuration.h>
#include <DracoBip/DracoBip_Definition.h>
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

BodyTask::BodyTask(const RobotSystem* robot):KinTask(3),
    robot_sys_(robot)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, dracobip::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
    sp_ = DracoBip_StateProvider::getStateProvider();
}

BodyTask::~BodyTask(){}

bool BodyTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

    // (Rx, Ry, Rz)
    dynacore::Quaternion des_ori;
    des_ori.x() = (*pos_cmd)[0];
    des_ori.y() = (*pos_cmd)[1];
    des_ori.z() = (*pos_cmd)[2];
    des_ori.w() = (*pos_cmd)[3];

    dynacore::Quaternion body_ori;
    //robot_sys_->getOri(dracobip_link::torso, body_ori);
    body_ori.x() = sp_->Q_[dracobip_joint::virtual_Rx];
    body_ori.y() = sp_->Q_[dracobip_joint::virtual_Ry];
    body_ori.z() = sp_->Q_[dracobip_joint::virtual_Rz];
    body_ori.w() = sp_->Q_[dracobip_joint::virtual_Rw];

    dynacore::Quaternion ori_err = dynacore::QuatMultiply(des_ori, body_ori.inverse());

    dynacore::Quaternion torso_ori;
    robot_sys_->getOri(dracobip_link::torso, torso_ori);

    //dynacore::pretty_print(body_ori, std::cout, "base ori");
    //dynacore::pretty_print(torso_ori, std::cout, "torso ori");
    
    dynacore::Vect3 ori_err_so3;
    dynacore::convert(ori_err, ori_err_so3);

    // Rx, Ry
    for(int i(0); i<2; ++i){
        pos_err_[i] = ori_err_so3[i];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }

    // (Z)
    dynacore::Vect3 body_pos;
    robot_sys_->getPos(dracobip_link::torso, body_pos);

    //for(int i(0); i<3; ++i){
        //pos_err_[i+3] =(*pos_cmd)[i+4] - body_pos[i];
        //vel_des_[i+3] = vel_des[i+3];
        //acc_des_[i+3] = acc_des[i+3];
    //}
    //pos_err_[3] =(*pos_cmd)[4] - sp_->Q_[dracobip_joint::virtual_X];
    //pos_err_[4] =(*pos_cmd)[6] - body_pos[2];
    pos_err_[2] =(*pos_cmd)[6] - sp_->Q_[dracobip_joint::virtual_Z];

    vel_des_[2] = vel_des[5];
    acc_des_[2] = acc_des[5];

    //vel_des_[4] = vel_des[5];
    //acc_des_[4] = acc_des[5];


    //printf("[Stance Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool BodyTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(dracobip_link::torso, Jtmp);
    // Rx, Ry, Rz
    Jt_.block(0,0, 2, dracobip::num_qdot) = Jtmp.block(0,0, 2, dracobip::num_qdot);
    // Z
    //Jt_.block(4,0, 1, dracobip::num_qdot) = Jtmp.block(5,0, 1, dracobip::num_qdot);

    // TEST
    //Jt_.setZero();
    //Jt_(0, 3) = 1.;
    //Jt_(1, 4) = 1.;
    //Jt_(3, 0) = 1.;
    Jt_(2, 2) = 1.;

    return true;
}

bool BodyTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}

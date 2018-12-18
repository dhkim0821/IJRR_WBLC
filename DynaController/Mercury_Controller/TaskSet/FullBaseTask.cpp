#include "FullBaseTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

FullBaseTask::FullBaseTask(const RobotSystem* robot):KinTask(6),
    robot_sys_(robot)
{
  sp_ = Mercury_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
    pos_err_.setZero();
    vel_des_.setZero();
    acc_des_.setZero();
}

FullBaseTask::~FullBaseTask(){}

bool FullBaseTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){

    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
    pos_err_[0] = (*pos_cmd)[0] - sp_->Q_[mercury_joint::virtual_Z];

    // Orientation
    dynacore::Quaternion curr_quat;
    curr_quat.w() = sp_->Q_[mercury::num_qdot];
    curr_quat.x() = sp_->Q_[3];
    curr_quat.y() = sp_->Q_[4];
    curr_quat.z() = sp_->Q_[5];

    dynacore::Quaternion des_quat;
    des_quat.x() = (*pos_cmd)[1];
    des_quat.y() = (*pos_cmd)[2];
    des_quat.z() = (*pos_cmd)[3];
    des_quat.w() = (*pos_cmd)[4];

    dynacore::Quaternion err_quat = 
        dynacore::QuatMultiply(des_quat, curr_quat.inverse());

    dynacore::Vect3 ori_err;
    dynacore::convert(err_quat, ori_err);

    pos_err_[1] = ori_err[0];
    pos_err_[2] = ori_err[1];

    //dynacore::pretty_print(pos_err_, std::cout, "pos_err");
     //dynacore::pretty_print(vel_des_, std::cout, "vel_des");
     //dynacore::pretty_print(acc_des_, std::cout, "acc_des");
     //dynacore::pretty_print(Jt_, std::cout, "Jt");
    return true;
}

bool FullBaseTask::_UpdateTaskJacobian(){
    robot_sys_->getFullJacobian(mercury_link::body, Jt_);
    Jt_.block(0,3, 3,3) = dynacore::Matrix::Identity(3,3);
    return true;
}

bool FullBaseTask::_UpdateTaskJDotQdot(){
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
    return true;
}

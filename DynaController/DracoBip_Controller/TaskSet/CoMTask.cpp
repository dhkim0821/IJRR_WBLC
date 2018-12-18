#include "CoMTask.hpp"
// Task consist of virtual joint (6) 
// (X, Y, Z), (Rx, Ry, Rz)

#include <Configuration.h>
#include <DracoBip/DracoBip_Definition.h>
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>

CoMTask::CoMTask(const RobotSystem* robot):KinTask(6),
    robot_sys_(robot)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, dracobip::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

CoMTask::~CoMTask(){}

bool CoMTask::_UpdateCommand(void* pos_des,
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
    robot_sys_->getOri(dracobip_link::torso, body_ori);
    dynacore::Quaternion ori_err = dynacore::QuatMultiply(des_ori, body_ori.inverse());
    
    dynacore::Vect3 ori_err_so3;
    dynacore::convert(ori_err, ori_err_so3);

    for(int i(0); i<3; ++i){
        pos_err_[i] = ori_err_so3[i];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }

    // (X, Y, Z)
    dynacore::Vect3 com_pos;
    robot_sys_->getCoMPosition(com_pos);

    for(int i(0); i<3; ++i){
        pos_err_[i+3] =(*pos_cmd)[i+4] - com_pos[i];
        vel_des_[i+3] = vel_des[i+3];
        acc_des_[i+3] = acc_des[i+3];
    }
 
    //printf("[Stance Task]\n");
     //dynacore::pretty_print(com_pos, std::cout, "com_pos");
     //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
     //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
     //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool CoMTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(dracobip_link::torso, Jtmp);
    Jt_ = Jtmp;
    robot_sys_->getCoMJacobian(Jtmp);
    Jt_.block(3, 0, 3, dracobip::num_qdot) = Jtmp;
    return true;
}

bool CoMTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}

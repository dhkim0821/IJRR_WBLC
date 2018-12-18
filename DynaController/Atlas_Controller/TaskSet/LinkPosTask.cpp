#include "LinkPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <Atlas/Atlas_Definition.h>
#include <Utils/utilities.hpp>
#include <Atlas/Atlas_Model.hpp>

LinkPosTask::LinkPosTask(const RobotSystem* robot, int link_idx):KinTask(3),
    robot_sys_(robot), link_idx_(link_idx)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, atlas::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

LinkPosTask::~LinkPosTask(){}

bool LinkPosTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vect3* pos_cmd = (dynacore::Vect3*)pos_des;
    dynacore::Vect3 link_pos;

    robot_sys_->getPos(link_idx_, link_pos);
    
    // X, Y, Z
    for(int i(0); i<3; ++i){
        pos_err_[i] = (*pos_cmd)[i] - link_pos[i];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }
    //printf("[Link Pos Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool LinkPosTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    Jt_ = Jtmp.block(3,0, 3, atlas::num_qdot);
    return true;
}

bool LinkPosTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.tail(3);
    return true;
}

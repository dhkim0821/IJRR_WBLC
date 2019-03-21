#include "LinkHeightTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <NAO/NAO_Definition.h>
#include <Utils/utilities.hpp>
#include <NAO/NAO_Model.hpp>

LinkHeightTask::LinkHeightTask(const RobotSystem* robot, int link_idx):KinTask(1),
    robot_sys_(robot), link_idx_(link_idx)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, nao::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

LinkHeightTask::~LinkHeightTask(){}

bool LinkHeightTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vect3* pos_cmd = (dynacore::Vect3*)pos_des;
    dynacore::Vect3 link_pos;

    robot_sys_->getPos(link_idx_, link_pos);
    
    // X, Y, Z
    for(int i(0); i<1; ++i){
        pos_err_[i] = (*pos_cmd)[i + 2] - link_pos[i + 2];
        vel_des_[i] = vel_des[i + 2];
        acc_des_[i] = acc_des[i + 2];
    }
    //printf("[Link Height Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");
    //dynacore::pretty_print(JtDotQdot_, std::cout, "JtDotQdot");

    return true;
}

bool LinkHeightTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    Jt_ = Jtmp.block(5,0, 1, nao::num_qdot);
    return true;
}

bool LinkHeightTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.tail(1);
    return true;
}

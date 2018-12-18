#include "LinkPosSelectTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>

LinkPosSelectTask::LinkPosSelectTask(const RobotSystem* robot, int link_idx, int selected_dim):
    KinTask(1),
    robot_sys_(robot), link_idx_(link_idx),
    selected_dim_(selected_dim)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, valkyrie::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

LinkPosSelectTask::~LinkPosSelectTask(){}

bool LinkPosSelectTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vect3* pos_cmd = (dynacore::Vect3*)pos_des;
    dynacore::Vect3 link_pos;

    robot_sys_->getPos(link_idx_, link_pos);

    pos_err_[0] = (*pos_cmd)[selected_dim_] - link_pos[selected_dim_];
    vel_des_[0] = vel_des[selected_dim_];
    acc_des_[0] = acc_des[selected_dim_];

    //printf("[Link Pos Select Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(link_pos, std::cout, "link_pos");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool LinkPosSelectTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    Jt_ = Jtmp.block(3 + selected_dim_,0, 1, valkyrie::num_qdot);
    return true;
}

bool LinkPosSelectTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.segment(3+selected_dim_,1);
    return true;
}

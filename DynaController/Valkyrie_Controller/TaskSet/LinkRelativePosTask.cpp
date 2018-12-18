#include "LinkRelativePosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>

LinkRelativePosTask::LinkRelativePosTask(
        const RobotSystem* robot, int link_idx, int ref_link_idx):
    KinTask(3),
    robot_sys_(robot), link_idx_(link_idx), ref_link_idx_(ref_link_idx)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, valkyrie::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

LinkRelativePosTask::~LinkRelativePosTask(){}

bool LinkRelativePosTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vect3* pos_cmd = (dynacore::Vect3*)pos_des;
    dynacore::Vect3 link_pos;
    dynacore::Vect3 ref_pos;

    robot_sys_->getPos(link_idx_, link_pos);
    robot_sys_->getPos(ref_link_idx_, link_pos);
    
    // X, Y
    for(int i(0); i<2; ++i){
        pos_err_[i] = (*pos_cmd)[i] - (link_pos[i] - ref_pos[i]);
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }
    pos_err_[2] = (*pos_cmd)[2] - link_pos[2];
    vel_des_[2] = vel_des[2];
    acc_des_[2] = acc_des[2];
     //printf("[Link Pos Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool LinkRelativePosTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp, Jtmp_ref;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    robot_sys_->getFullJacobian(ref_link_idx_, Jtmp_ref);
    Jt_ = Jtmp.block(3,0, 3, valkyrie::num_qdot);
    Jt_.block(0,0, 2, valkyrie::num_qdot) -= Jtmp_ref.block(3,0,2, valkyrie::num_qdot);
    return true;
}

bool LinkRelativePosTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.tail(3);
    return true;
}

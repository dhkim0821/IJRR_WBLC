#include "LinkGlobalYZPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>

LinkGlobalYZPosTask::LinkGlobalYZPosTask(const RobotSystem* robot, int link_idx):
    KinTask(2),
    robot_sys_(robot), link_idx_(link_idx)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, valkyrie::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
    sp_ = Valkyrie_StateProvider::getStateProvider();
}

LinkGlobalYZPosTask::~LinkGlobalYZPosTask(){}

bool LinkGlobalYZPosTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
    dynacore::Vect3 link_pos;

    robot_sys_->getPos(link_idx_, link_pos);
    
    // X, Y, Z
    for(int i(0); i<2; ++i){
        pos_err_[i] = (*pos_cmd)[i] - (link_pos[i+1] + sp_->global_pos_local_[i+1]);
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

bool LinkGlobalYZPosTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    Jt_ = Jtmp.block(4,0, 2, valkyrie::num_qdot);
    return true;
}

bool LinkGlobalYZPosTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.tail(2);
    return true;
}

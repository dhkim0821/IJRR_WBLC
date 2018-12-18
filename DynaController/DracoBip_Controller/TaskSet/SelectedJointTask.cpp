#include "SelectedJointTask.hpp"

#include <DracoBip/DracoBip_Definition.h>
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

SelectedJointTask::SelectedJointTask(const std::vector<int> & selected_jidx):
    KinTask(selected_jidx.size())
{
    selected_jidx_ = selected_jidx;
    Jt_ = dynacore::Matrix::Zero(dim_task_, dracobip::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
    
    for(int i(0); i<selected_jidx_.size(); ++i){
        Jt_(i, selected_jidx_[i]) = 1.;
    }
    sp_ = DracoBip_StateProvider::getStateProvider();
}

SelectedJointTask::~SelectedJointTask(){}

bool SelectedJointTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

    for(int i(0); i<selected_jidx_.size(); ++i){
        pos_err_[i] = ((*pos_cmd)[i] - sp_->Q_[selected_jidx_[i]]);
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }
    //printf("[Selected Joint Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool SelectedJointTask::_UpdateTaskJacobian(){
    return true;
}

bool SelectedJointTask::_UpdateTaskJDotQdot(){
    return true;
}

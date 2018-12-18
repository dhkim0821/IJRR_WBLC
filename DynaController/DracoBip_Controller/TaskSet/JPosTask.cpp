#include "JPosTask.hpp"
#include <Configuration.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <Utils/utilities.hpp>

JPosTask::JPosTask():Task(dracobip::num_act_joint),
                     Kp_vec_(dracobip::num_act_joint),
                     Kd_vec_(dracobip::num_act_joint)
{
  Kp_vec_.setZero();
  Kd_vec_.setZero();
  for(int i(0); i<dracobip::num_act_joint; ++i){
    Kp_vec_[i] = 150.;
    Kd_vec_[i] = 3.;
  }
  sp_ = DracoBip_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix(dracobip::num_act_joint, dracobip::num_qdot);
  JtDotQdot_ = dynacore::Vector(dracobip::num_act_joint);
}

JPosTask::~JPosTask(){}

bool JPosTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  for(int i(0); i<dracobip::num_act_joint; ++i){
    op_cmd_[i] = acc_des[i] 
        + Kp_vec_[i] * ((*pos_cmd)[i] - sp_->Q_[dracobip::num_virtual + i]) 
        + Kd_vec_[i] * (vel_des[i] - sp_->Qdot_[dracobip::num_virtual + i]);
  }
   //dynacore::pretty_print(acc_des, std::cout, "acc_des");
   //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(sp_->Q_, std::cout, "config");

  return true;
}

bool JPosTask::_UpdateTaskJacobian(){
  Jt_.setZero();
  (Jt_.block(0, dracobip::num_virtual, dracobip::num_act_joint, dracobip::num_act_joint)).setIdentity();

  return true;
}

bool JPosTask::_UpdateTaskJDotQdot(){
  JtDotQdot_.setZero();
  return true;
}

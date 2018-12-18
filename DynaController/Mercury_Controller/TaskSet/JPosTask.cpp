#include "JPosTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/utilities.hpp>

JPosTask::JPosTask():Task(mercury::num_act_joint),
                     Kp_vec_(mercury::num_act_joint),
                     Kd_vec_(mercury::num_act_joint)
{
  Kp_vec_.setZero();
  Kd_vec_.setZero();
  for(int i(0); i<mercury::num_act_joint; ++i){
    Kp_vec_[i] = 150.;
    Kd_vec_[i] = 3.;
  }
  sp_ = Mercury_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix(mercury::num_act_joint, mercury::num_qdot);
  JtDotQdot_ = dynacore::Vector(mercury::num_act_joint);
}

JPosTask::~JPosTask(){}

bool JPosTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  for(int i(0); i<mercury::num_act_joint; ++i){
    op_cmd_[i] = acc_des[i] 
        + Kp_vec_[i] * ((*pos_cmd)[i] - sp_->Q_[mercury::num_virtual + i]) 
        + Kd_vec_[i] * (vel_des[i] - sp_->Qdot_[mercury::num_virtual + i]);
  }
   //dynacore::pretty_print(acc_des, std::cout, "acc_des");
   //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(sp_->Q_, std::cout, "config");

  return true;
}

bool JPosTask::_UpdateTaskJacobian(){
  Jt_.setZero();
  (Jt_.block(0, mercury::num_virtual, mercury::num_act_joint, mercury::num_act_joint)).setIdentity();

  return true;
}

bool JPosTask::_UpdateTaskJDotQdot(){
  JtDotQdot_.setZero();
  return true;
}

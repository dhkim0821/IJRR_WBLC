#include "JPosTask.hpp"
#include <Configuration.h>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Utils/utilities.hpp>

JPosTask::JPosTask():KinTask(valkyrie::num_act_joint)
{
  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix::Zero(valkyrie::num_act_joint, valkyrie::num_qdot);
  (Jt_.block(0, valkyrie::num_virtual, 
             valkyrie::num_act_joint, valkyrie::num_act_joint)).setIdentity();
  JtDotQdot_ = dynacore::Vector::Zero(valkyrie::num_act_joint);
}

JPosTask::~JPosTask(){}

bool JPosTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  for(int i(0); i<valkyrie::num_act_joint; ++i){
        pos_err_[i] = (*pos_cmd)[i] - sp_->Q_[i + valkyrie::num_virtual];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
  }
   //dynacore::pretty_print(acc_des, std::cout, "acc_des");
   //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(sp_->Q_, std::cout, "config");

  return true;
}

bool JPosTask::_UpdateTaskJacobian(){
  return true;
}

bool JPosTask::_UpdateTaskJDotQdot(){
  return true;
}

#include "FixedBodyContact.hpp"
#include <NAO/NAO_Model.hpp>
#include <NAO/NAO_Definition.h>
#include <NAO_Controller/NAO_StateProvider.hpp>
#include <Utils/utilities.hpp>

FixedBodyContact::FixedBodyContact(RobotSystem* robot):WBLC_ContactSpec(6)
{
  robot_sys_ = robot;
  sp_ = NAO_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix::Zero(dim_contact_, nao::num_qdot);
}
FixedBodyContact::~FixedBodyContact(){ }

bool FixedBodyContact::_UpdateJc(){
  for(int i(0);i<dim_contact_; ++i)  Jc_(i,i) = 1.;
  return true;
}

bool FixedBodyContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateUf(){
  Uf_ = dynacore::Matrix::Zero(1, dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(1);
  return true;
}

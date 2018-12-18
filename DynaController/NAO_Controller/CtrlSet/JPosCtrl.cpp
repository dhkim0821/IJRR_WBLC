#include "JPosCtrl.hpp"
#include <Utils/utilities.hpp>
#include <NAO/NAO_Definition.h>
#include "../NAO_Exam_StateProvider.hpp"

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    ini_jpos_(nao::num_act_joint)
{
    sp_ = NAO_Exam_StateProvider::getStateProvider();
}

JPosCtrl::~JPosCtrl(){
}

void JPosCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_  += nao::servo_rate;
  gamma.setZero();

  int jidx (7);
  double amp(0.);
  double omega(2.*M_PI * 1.0);

  dynacore::Vector jpos_des = ini_jpos_;
  dynacore::Vector jvel_des(nao::num_act_joint);
  jvel_des.setZero();

  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);

  //gamma = 100.0 * (jpos_des - sp_->q_.segment(6, nao::num_act_joint)) 
      //+ 1.0 * ( jvel_des - sp_->qdot_.segment(6, nao::num_act_joint) );

  dynacore::Vector jacc = 100.0 * (jpos_des - sp_->q_.segment(6, nao::num_act_joint)) 
                              + 2. * ( jvel_des - sp_->qdot_.tail(nao::num_act_joint) );
  dynacore::Vector force_cmd = A_.block(nao::num_virtual, nao::num_virtual, 
          nao::num_act_joint, nao::num_act_joint) * jacc;
  dynacore::Matrix jr, jl;
  robot_sys_->getFullJacobian(nao_link::r_ankle, jr);
  robot_sys_->getFullJacobian(nao_link::l_ankle, jl);

  dynacore::Matrix Jc(12, nao::num_qdot);
  Jc.block(0, 0, 6, nao::num_qdot) = jr;
  Jc.block(6, 0, 6, nao::num_qdot) = jl;
  dynacore::Matrix Jc_inv;
  _DynConsistent_Inverse(Jc, Jc_inv); 

  dynacore::Matrix eye(nao::num_qdot, nao::num_qdot);
  eye.setIdentity();
  dynacore::Matrix Nc = eye - Jc_inv * Jc;

    dynacore::Matrix U(nao::num_act_joint, nao::num_qdot);
    U.setZero();
    U.block(0, nao::num_virtual, nao::num_act_joint, nao::num_act_joint).setIdentity();
    dynacore::Matrix UNc = U * Nc;
    dynacore::Matrix UNcBar;
    _DynConsistent_Inverse(UNc, UNcBar);

    dynacore::Vector grav_null = UNcBar.transpose() * (Nc.transpose() * 
        (grav_ ) );



  gamma += grav_null.tail(nao::num_act_joint);

  //dynacore::pretty_print(gamma, std::cout, "gamma");
  //dynacore::pretty_print(ini_jpos_, std::cout, "ini_jpos");
  //dynacore::pretty_print(sp_->q_, std::cout, "q");
  //dynacore::pretty_print(A_, std::cout, "A");
  //dynacore::pretty_print(sp_->qdot_, std::cout, "qdot");
  //dynacore::pretty_print(grav_, std::cout, "grav");
  //dynacore::pretty_print(grav_null, std::cout, "grav null");

  _PostProcessing_Command();
}

void JPosCtrl::FirstVisit(){
    for(int i(0); i<nao::num_act_joint; ++i) ini_jpos_[i] = sp_->q_[i+6];
}

void JPosCtrl::LastVisit(){
}

bool JPosCtrl::EndOfPhase(){
    return false;
}

void JPosCtrl::CtrlInitialization(const std::string & setting_file_name){
}

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Utils/wrap_eigen.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <RobotSystems/RobotSystem.hpp>
#include <Task.hpp>
#include <ContactSpec.hpp>

class Controller{
public:
  Controller(const RobotSystem* robot):robot_sys_(robot),state_machine_time_(0.){}
  virtual ~Controller(){}

  virtual void OneStep(void* command) = 0;
  virtual void FirstVisit() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfPhase() = 0;
  virtual void CtrlInitialization(const std::string & setting_file_name) = 0;

protected:
  void _DynConsistent_Inverse(const dynacore::Matrix & J, dynacore::Matrix & Jinv){
      dynacore::Matrix Jtmp(J * Ainv_ * J.transpose());
      dynacore::Matrix Jtmp_inv;
      dynacore::pseudoInverse(Jtmp, 0.0001, Jtmp_inv, 0);
      Jinv = Ainv_ * J.transpose() * Jtmp_inv;
  }

  void _PreProcessing_Command(){
      robot_sys_->getMassInertia(A_);
      robot_sys_->getInverseMassInertia(Ainv_);
      robot_sys_->getGravity(grav_);
      robot_sys_->getCoriolis(coriolis_);

      task_list_.clear();
      contact_list_.clear();
  }

  void _PostProcessing_Command(){
      for(int i(0); i<task_list_.size(); ++i){ task_list_[i]->UnsetTask(); }
      for(int i(0); i<contact_list_.size(); ++i){ contact_list_[i]->UnsetContact(); }
  }

  const RobotSystem* robot_sys_;

  dynacore::Matrix A_;
  dynacore::Matrix Ainv_;
  dynacore::Vector grav_;
  dynacore::Vector coriolis_;

  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

  double state_machine_time_;
};

#endif

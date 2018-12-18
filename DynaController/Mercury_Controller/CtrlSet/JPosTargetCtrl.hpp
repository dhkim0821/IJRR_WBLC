#ifndef JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL
#define JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBLC_ContactSpec;
class WBLC;
class WBLC_ExtraData;

class JPosTargetCtrl: public Controller{
public:
  JPosTargetCtrl(RobotSystem* );
  virtual ~JPosTargetCtrl();

  virtual void OneStep(void* _cmd);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setMovingTime(double time) { end_time_ = time; }
  void setTargetPosition(const std::vector<double> & jpos);
  void setTargetPosition(const dynacore::Vector & jpos){ jpos_target_ = jpos; }

  void setInitialPosition(const std::vector<double> & jpos);

protected:
  double end_time_;
  bool b_external_initial_pos_set_;
  int dim_contact_;

  WBLC_ContactSpec* fixed_body_contact_;
  WBLC* wblc_;
  WBLC_ExtraData* wblc_data_;
 
  dynacore::Vector jpos_ini_;
  dynacore::Vector jpos_target_;

  dynacore::Vector des_jpos_;
  dynacore::Vector des_jvel_;
  dynacore::Vector des_jacc_;
 
  dynacore::Vector Kp_, Kd_;
 
  void _task_setup();
  void _contact_setup();
  void _compute_torque_wblc(dynacore::Vector & gamma);

  Mercury_StateProvider* sp_;
  double ctrl_start_time_;
};

#endif

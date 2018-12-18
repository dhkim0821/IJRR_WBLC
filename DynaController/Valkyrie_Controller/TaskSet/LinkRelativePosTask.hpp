#ifndef KINEMATICS_LINK_RELATIVE_POS_TASK
#define KINEMATICS_LINK_RELATIVE_POS_TASK

// (X, Y, Z)
// Height is defined in global
#include <WBLC/KinTask.hpp>

class RobotSystem;

class LinkRelativePosTask: public KinTask{
public:
  LinkRelativePosTask(const RobotSystem*, int link_idx, int ref_link_idx);
  virtual ~LinkRelativePosTask();

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true; }

  int link_idx_;
  int ref_link_idx_;
  const RobotSystem* robot_sys_;
};

#endif

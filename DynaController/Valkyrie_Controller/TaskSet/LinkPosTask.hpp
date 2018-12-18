#ifndef KINEMATICS_LINK_POS_TASK
#define KINEMATICS_LINK_POS_TASK

// (X, Y, Z)
#include <WBLC/KinTask.hpp>

class RobotSystem;

class LinkPosTask: public KinTask{
public:
  LinkPosTask(const RobotSystem*, int link_idx, bool virtual_depend = true);
  virtual ~LinkPosTask();

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
  bool virtual_depend_;
  const RobotSystem* robot_sys_;
};

#endif

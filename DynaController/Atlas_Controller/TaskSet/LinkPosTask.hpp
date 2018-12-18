#ifndef KINEMATICS_LINK_POS_TASK_Atlas
#define KINEMATICS_LINK_POS_TASK_Atlas

// (X, Y, Z)
#include <WBLC/KinTask.hpp>

class RobotSystem;

class LinkPosTask: public KinTask{
public:
  LinkPosTask(const RobotSystem*, int link_idx);
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
  const RobotSystem* robot_sys_;
};

#endif

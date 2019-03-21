#ifndef KINEMATICS_LINK_ORIENTATION_TASK
#define KINEMATICS_LINK_ORIENTATION_TASK
// (Rx, Ry, Rz)
#include <WBLC/KinTask.hpp>

class RobotSystem;

class LinkOriTask: public KinTask{
public:
  LinkOriTask(const RobotSystem*, int link_idx);
  virtual ~LinkOriTask();

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

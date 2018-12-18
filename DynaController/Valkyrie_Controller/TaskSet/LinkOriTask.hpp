#ifndef KINEMATICS_LINK_ORIENTATION_TASK_Atlas
#define KINEMATICS_LINK_ORIENTATION_TASK_Atlas
// (Rx, Ry, Rz)
#include <WBLC/KinTask.hpp>

class RobotSystem;

class LinkOriTask: public KinTask{
public:
  LinkOriTask(const RobotSystem*, int link_idx, bool virtual_depend = true);
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
  bool virtual_depend_; 
  const RobotSystem* robot_sys_;
};

#endif

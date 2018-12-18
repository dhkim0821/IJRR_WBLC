#ifndef KINEMATICS_CoM_TASK
#define KINEMATICS_CoM_TASK
// Task consist of virtual joint (6)
// (Rx, Ry, Rz), (X, Y, Z)
#include <WBLC/KinTask.hpp>

class DracoBip_StateProvider;
class RobotSystem;

class CoMTask: public KinTask{
public:
  CoMTask(const RobotSystem*);
  virtual ~CoMTask();

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

  const RobotSystem* robot_sys_;
  DracoBip_StateProvider* sp_;
  int stance_leg_jidx_;
};

#endif

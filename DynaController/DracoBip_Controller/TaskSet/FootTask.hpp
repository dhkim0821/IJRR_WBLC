#ifndef KINEMATICS_FOOT_TASK_DRACO_BIPED
#define KINEMATICS_FOOT_TASK_DRACO_BIPED

#include <WBLC/KinTask.hpp>

class DracoBip_StateProvider;
class RobotSystem;

// FootOri_{Rz}, Foot (x, y, z)
class FootTask: public KinTask{
public:
  FootTask(const RobotSystem*, int swing_foot);
  virtual ~FootTask();

protected:
  int swing_foot_;
  int stance_foot_;

  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true;}

  DracoBip_StateProvider* sp_;
  const RobotSystem* robot_sys_;
};

#endif

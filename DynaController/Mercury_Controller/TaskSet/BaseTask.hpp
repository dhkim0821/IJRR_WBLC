#ifndef WBDC_BODY_HEIGHT_ORIENTATION_TASK
#define WBDC_BODY_HEIGHT_ORIENTATION_TASK

#include <WBLC/KinTask.hpp>

class Mercury_StateProvider;
class RobotSystem;

class BaseTask: public KinTask{
public:
  BaseTask(const RobotSystem*); // Z, Rx, Ry
  virtual ~BaseTask();

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true;}

  const RobotSystem* robot_sys_;
  Mercury_StateProvider* sp_;
};

#endif

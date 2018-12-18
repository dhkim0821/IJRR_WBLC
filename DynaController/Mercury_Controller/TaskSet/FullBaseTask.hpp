#ifndef WBDC_FULL_BASE_TASK
#define WBDC_FULL_BASE_TASK

#include <WBLC/KinTask.hpp>

class Mercury_StateProvider;
class RobotSystem;

class FullBaseTask: public KinTask{
public:
  FullBaseTask(const RobotSystem*);
  virtual ~FullBaseTask();

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

#ifndef SELECTED_JOINT_TASK
#define SELECTED_JOINT_TASK

#include <WBLC/KinTask.hpp>

class DracoBip_StateProvider;
class RobotSystem;

class SelectedJointTask: public KinTask{
public:
  SelectedJointTask(const std::vector<int> & selected_jidx);
  virtual ~SelectedJointTask();

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

  std::vector<int> selected_jidx_;
  DracoBip_StateProvider* sp_;
};

#endif

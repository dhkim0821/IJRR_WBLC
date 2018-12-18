#ifndef SELECTED_JOINT_TASK_Atlas
#define SELECTED_JOINT_TASK_Atlas

#include <WBLC/KinTask.hpp>

class Atlas_StateProvider;
class RobotSystem;

class SelectedJPosTask: public KinTask{
public:
  SelectedJPosTask(const std::vector<int> & selected_jidx);
  virtual ~SelectedJPosTask();

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
  Atlas_StateProvider* sp_;
};

#endif

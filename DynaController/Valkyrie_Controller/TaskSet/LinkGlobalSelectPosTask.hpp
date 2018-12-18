#ifndef KINEMATICS_LINK_GLOBAL_POS_Select_TASK
#define KINEMATICS_LINK_GLOBAL_POS_Select_TASK

// selected dim
// 0: X, 1:Y, 2:Z

#include <WBLC/KinTask.hpp>

class RobotSystem;
class Valkyrie_StateProvider;

class LinkGlobalSelectPosTask: public KinTask{
public:
  LinkGlobalSelectPosTask(const RobotSystem*, int link_idx, int selected_dim);
  virtual ~LinkGlobalSelectPosTask();

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
  int selected_dim_;
  const RobotSystem* robot_sys_;
  const Valkyrie_StateProvider* sp_;
};

#endif

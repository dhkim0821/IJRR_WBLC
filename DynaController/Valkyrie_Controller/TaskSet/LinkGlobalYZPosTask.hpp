#ifndef KINEMATICS_LINK_GLOBAL_POS_YZ_TASK
#define KINEMATICS_LINK_GLOBAL_POS_YZ_TASK

// (Y, Z)
#include <WBLC/KinTask.hpp>

class RobotSystem;
class Valkyrie_StateProvider;

class LinkGlobalYZPosTask: public KinTask{
public:
  LinkGlobalYZPosTask(const RobotSystem*, int link_idx);
  virtual ~LinkGlobalYZPosTask();

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
  const Valkyrie_StateProvider* sp_;
};

#endif

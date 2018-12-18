#ifndef JPOS_TASK_Atlas
#define JPOS_TASK_Atlas

#include <WBLC/KinTask.hpp>

class Atlas_StateProvider;

class JPosTask: public KinTask{
public:
  JPosTask();
  virtual ~JPosTask();

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

  Atlas_StateProvider* sp_;
};

#endif

#ifndef JPOS_TASK_NAO
#define JPOS_TASK_NAO

#include <WBLC/KinTask.hpp>

class NAO_StateProvider;

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

  NAO_StateProvider* sp_;
};

#endif

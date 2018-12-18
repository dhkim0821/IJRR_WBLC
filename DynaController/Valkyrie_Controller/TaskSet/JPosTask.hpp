#ifndef JPOS_TASK_Valkyrie
#define JPOS_TASK_Valkyrie

#include <WBLC/KinTask.hpp>

class Valkyrie_StateProvider;

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

  Valkyrie_StateProvider* sp_;
};

#endif

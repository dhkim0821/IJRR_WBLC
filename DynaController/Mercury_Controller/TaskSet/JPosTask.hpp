#ifndef WBDC_JPOS_TASK
#define WBDC_JPOS_TASK

#include <Task.hpp>

class Mercury_StateProvider;

class JPosTask: public Task{
public:
  JPosTask();
  virtual ~JPosTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

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

  Mercury_StateProvider* sp_;
};

#endif

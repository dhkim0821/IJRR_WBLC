#ifndef TEST_H
#define TEST_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class Controller;
class RobotSystem;

class Test{
public:
  Test(RobotSystem* );
  virtual ~Test();

  virtual void TestInitialization() = 0;
  void getCommand(void* _command);

  int getPhase(){ return phase_;}

protected:
  virtual int _NextPhase(const int & phase) = 0;

  bool b_first_visit_;
  int phase_;
  std::vector<Controller*> state_list_;

};


#endif

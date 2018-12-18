#ifndef BODY_CONTROL_TEST
#define BODY_CONTROL_TEST

#include <Test.hpp>

class RobotSystem;

enum BodyCtrlPhase{
  BC_initial_jpos = 0,
  BC_lift_up = 1,
  BC_body_ctrl = 2,
  NUM_BC_PHASE
};

class BodyCtrlTest: public Test{
public:
  BodyCtrlTest(RobotSystem* );
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_ctrl_;

};

#endif

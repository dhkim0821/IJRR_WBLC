#ifndef BODY_CONTROL_TEST_ATLAS
#define BODY_CONTROL_TEST_ATLAS

#include <Test.hpp>

class RobotSystem;

enum BodyCtrlPhase{
  BDCTRL_lift_up = 0,
  BDCTRL_body_ctrl = 1,
  NUM_BDCTRL_PHASE
};

class BodyCtrlTest: public Test{
public:
  BodyCtrlTest(RobotSystem* );
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller* body_up_ctrl_;
  Controller* body_ctrl_;

};

#endif

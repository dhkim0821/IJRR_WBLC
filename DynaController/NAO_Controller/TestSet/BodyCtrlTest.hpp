#ifndef BODY_CONTROL_TEST_NAO
#define BODY_CONTROL_TEST_NAO

#include <Test.hpp>

class RobotSystem;

enum BodyCtrlPhase{
  BDCTRL_body_ctrl = 0,
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
  
  Controller* body_ctrl_;

};

#endif

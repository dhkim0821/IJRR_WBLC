#include "BodyCtrlTest.hpp"

#include <NAO_Controller/CtrlSet/BodyCtrl.hpp>
#include <NAO_Controller/NAO_DynaCtrl_Definition.h>

#include <ParamHandler/ParamHandler.hpp>
#include <NAO/NAO_Model.hpp>

BodyCtrlTest::BodyCtrlTest(RobotSystem* robot):Test(robot){
    phase_ = BodyCtrlPhase::BDCTRL_body_ctrl;
    state_list_.clear();

    body_ctrl_ = new BodyCtrl(robot);
    state_list_.push_back(body_ctrl_);

    _SettingParameter();

    printf("[Body Joint Position Control Test] Constructed\n");
}

BodyCtrlTest::~BodyCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyCtrlTest::TestInitialization(){
  // Yaml file name
  body_ctrl_->CtrlInitialization("CTRL_stance");
}

int BodyCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == BodyCtrlPhase::NUM_BDCTRL_PHASE) {
    return BodyCtrlPhase::BDCTRL_body_ctrl;
  }
  else return next_phase;
}

void BodyCtrlTest::_SettingParameter(){
  ParamHandler handler(NAOConfigPath"TEST_body_ctrl.yaml");
  double tmp;
  // Stance Time
  handler.getValue("body_stay_time", tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceTime(tmp);

  handler.getValue("body_height", tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceHeight(tmp);
}


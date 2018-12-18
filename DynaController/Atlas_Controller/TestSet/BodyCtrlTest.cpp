#include "BodyCtrlTest.hpp"

#include <Atlas_Controller/CtrlSet/BodyCtrl.hpp>
#include <Atlas_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <Atlas_Controller/Atlas_DynaCtrl_Definition.h>

#include <ParamHandler/ParamHandler.hpp>
#include <Atlas/Atlas_Model.hpp>

BodyCtrlTest::BodyCtrlTest(RobotSystem* robot):Test(robot){
    phase_ = BodyCtrlPhase::BDCTRL_body_ctrl;
    //phase_ = BodyCtrlPhase::BDCTRL_body_up_ctrl;
    state_list_.clear();

    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_ctrl_ = new BodyCtrl(robot);
    
    state_list_.push_back(body_up_ctrl_);
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
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_ctrl_->CtrlInitialization("CTRL_stance");
}

int BodyCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BDCTRL_PHASE) {
    return BodyCtrlPhase::BDCTRL_body_ctrl;
  }
  else return next_phase;
}

void BodyCtrlTest::_SettingParameter(){
  ParamHandler handler(AtlasConfigPath"TEST_body_ctrl.yaml");
  double tmp;
  // Stance Time
  handler.getValue("body_lifting_time", tmp);
  ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);
  handler.getValue("body_stay_time", tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceTime(tmp);

  handler.getValue("body_height", tmp);
  ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceHeight(tmp);
}


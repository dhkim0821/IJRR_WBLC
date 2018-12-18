#include "BodyCtrlTest.hpp"

#include <DracoBip_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <DracoBip_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <DracoBip_Controller/CtrlSet/BodyCtrl.hpp>
#include <DracoBip_Controller/CtrlSet/CoMCtrl.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

BodyCtrlTest::BodyCtrlTest(RobotSystem* robot):Test(robot){
  //phase_ = BodyCtrlPhase::BC_body_ctrl;
  phase_ = BodyCtrlPhase::BC_initial_jpos;
  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new DoubleContactTransCtrl(robot);
  body_ctrl_ = new BodyCtrl(robot);
  //body_ctrl_ = new CoMCtrl(robot);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_ctrl_);

  _SettingParameter();

  printf("[Body Control Test] Constructed\n");
}

BodyCtrlTest::~BodyCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyCtrlTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_ctrl_->CtrlInitialization("CTRL_stance");
}

int BodyCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BC_PHASE) {
    return BodyCtrlPhase::BC_body_ctrl;
  }
  else return next_phase;
}


void BodyCtrlTest::_SettingParameter(){
  ParamHandler handler(DracoBipConfigPath"TEST_body_ctrl.yaml");

  double tmp;
  std::vector<double> tmp_vec;
  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

  // Body Height
  handler.getValue("body_height", tmp);
  ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("body_lifting_time", tmp);
  ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("body_ctrl_time", tmp);
  ((BodyCtrl*)body_ctrl_)->setStanceTime(tmp);
}

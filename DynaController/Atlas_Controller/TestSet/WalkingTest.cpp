#include "WalkingTest.hpp"
#include <Atlas_Controller/Atlas_StateProvider.hpp>

#include <Atlas_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <Atlas_Controller/CtrlSet/BodyCtrl.hpp>
#include <Atlas_Controller/CtrlSet/SingleContactTransCtrl.hpp>
#include <Atlas_Controller/CtrlSet/BodyFootPlanningCtrl.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <Atlas/Atlas_Model.hpp>
#include <Atlas_Controller/Atlas_DynaCtrl_Definition.h>


WalkingTest::WalkingTest(RobotSystem* robot):Test(robot),
    num_step_(0)
{
    sp_ = Atlas_StateProvider::getStateProvider();
    sp_->stance_foot_ = atlas_link::leftFoot;
    sp_->global_pos_local_[1] = 0.15;

    robot_sys_ = robot;
    reversal_planner_ = new Reversal_LIPM_Planner();
    //phase_ = WkPhase::lift_up;
    phase_ = WkPhase::double_contact_1;

    state_list_.clear();

    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);
    // Right
    right_swing_start_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, atlas_link::rightFoot, false);
    right_swing_end_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, atlas_link::rightFoot, true);
    // Left
    left_swing_start_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, atlas_link::leftFoot, false);
    left_swing_end_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, atlas_link::leftFoot, true);
    // Swing Controller Selection
    right_swing_ctrl_ = 
        new BodyFootPlanningCtrl(robot_sys_, atlas_link::rightFoot, reversal_planner_);
    left_swing_ctrl_ = 
        new BodyFootPlanningCtrl(robot_sys_, atlas_link::leftFoot, reversal_planner_);


    _SettingParameter();

    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(right_swing_start_trans_ctrl_);
    state_list_.push_back(right_swing_ctrl_);
    state_list_.push_back(right_swing_end_trans_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(left_swing_start_trans_ctrl_);
    state_list_.push_back(left_swing_ctrl_);
    state_list_.push_back(left_swing_end_trans_ctrl_);

    printf("[Walking  Test] Constructed\n");
}

WalkingTest::~WalkingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void WalkingTest::TestInitialization(){
    // Planner
    reversal_planner_->PlannerInitialization(
            AtlasConfigPath"PLANNER_velocity_reversal");
    // Yaml file name
    body_fix_ctrl_->CtrlInitialization("CTRL_stance");
    // Transition
    right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
    left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
    // Swing
    right_swing_ctrl_->CtrlInitialization("CTRL_right_walking_swing");
    left_swing_ctrl_->CtrlInitialization("CTRL_left_walking_swing");
}

int WalkingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    // printf("next phase: %i\n", next_phase);

    if(phase == WkPhase::double_contact_1) {
        ++num_step_;
        sp_->stance_foot_ = atlas_link::leftFoot;

        // Global Frame Update
        dynacore::Vect3 next_local_frame_location;
        robot_sys_->getPos(atlas_link::leftFoot, next_local_frame_location);

        sp_->global_pos_local_ += next_local_frame_location;
        
        // dynacore::pretty_print(next_local_frame_location, std::cout, "landing loc");
        // printf("%i th step:\n", num_step_);
    }
    if(phase == WkPhase::double_contact_2){
        ++num_step_;
        sp_->stance_foot_ = atlas_link::rightFoot;

        // Global Frame Update
        dynacore::Vect3 next_local_frame_location;
        robot_sys_->getPos(atlas_link::rightFoot, next_local_frame_location);
        sp_->global_pos_local_ += next_local_frame_location;
        
        // dynacore::pretty_print(next_local_frame_location, std::cout, "landing loc");
        // printf("%i th step:\n", num_step_);
    }

    sp_->num_step_copy_ = num_step_;

    if(next_phase == WkPhase::NUM_WALKING_PHASE) {
        return WkPhase::double_contact_1;
    }
    else{ return next_phase; }
}

void WalkingTest::_SettingParameter(){
    // Setting Parameters
    ParamHandler handler(AtlasConfigPath"TEST_walking.yaml");

    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;

    //// Posture Setup
    // CoM Height
    handler.getValue("body_height", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

    ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

    ((SwingPlanningCtrl*)right_swing_ctrl_)->setStanceHeight(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setStanceHeight(tmp);
    ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

    //// Timing Setup
    handler.getValue("com_lifting_time", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

    // Stance Time
    handler.getValue("stance_time", tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyStanceTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyStanceTime(tmp);

    // Swing & prime Time
    handler.getValue("swing_time", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

    // Transition Time
    handler.getValue("st_transition_time", tmp);
    ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

    ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyTransitionTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyTransitionTime(tmp);
    //// Planner Setup
    handler.getBoolean("replanning", b_tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setReplanning(b_tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setReplanning(b_tmp);

    handler.getValue("double_stance_mix_ratio", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setDoubleStanceRatio(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setDoubleStanceRatio(tmp);

    handler.getValue("transition_phase_mix_ratio", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

    handler.getBoolean("contact_switch_check", b_tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setContactSwitchCheck(b_tmp);

    printf("[Walking Body Test] Complete to Setup Parameters\n");
}


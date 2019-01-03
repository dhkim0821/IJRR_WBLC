#include "WalkingConfigTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ConfigBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionConfigCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

#include <Mercury_Controller/CtrlSet/ConfigBodyFootPlanningCtrl.hpp>

WalkingConfigTest::WalkingConfigTest(RobotSystem* robot):Test(robot),
    num_step_(0)
{
    sp_ = Mercury_StateProvider::getStateProvider();
#if (CONFIG_INITIAL_SWING_FOOT == 0)
    sp_->stance_foot_ = mercury_link::leftFoot;
#else
    sp_->stance_foot_ = mercury_link::rightFoot;
#endif
    sp_->global_pos_local_[1] = 0.15;
    sp_->global_jjpos_local_[1] = 0.15;

    robot_sys_ = robot;
    reversal_planner_ = new Reversal_LIPM_Planner();
    phase_ = WkConfigPhase::initiation;

    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    config_body_fix_ctrl_ = new ConfigBodyCtrl(robot);
    // Right
    right_swing_start_trans_ctrl_ = 
        new TransitionConfigCtrl(robot, mercury_link::rightFoot, false);
    right_swing_end_trans_ctrl_ = 
        new TransitionConfigCtrl(robot, mercury_link::rightFoot, true);
    // Left
    left_swing_start_trans_ctrl_ = 
        new TransitionConfigCtrl(robot, mercury_link::leftFoot, false);
    left_swing_end_trans_ctrl_ = 
        new TransitionConfigCtrl(robot, mercury_link::leftFoot, true);


    _SettingParameter();

    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(config_body_fix_ctrl_);
    state_list_.push_back(right_swing_start_trans_ctrl_);
    state_list_.push_back(config_right_swing_ctrl_);
    state_list_.push_back(right_swing_end_trans_ctrl_);
    state_list_.push_back(config_body_fix_ctrl_);
    state_list_.push_back(left_swing_start_trans_ctrl_);
    state_list_.push_back(config_left_swing_ctrl_);
    state_list_.push_back(left_swing_end_trans_ctrl_);


    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_right_swing_ctrl_)->curr_foot_pos_des_), 
            VECT3, "rfoot_pos_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_left_swing_ctrl_)->curr_foot_pos_des_), 
            VECT3, "lfoot_pos_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_right_swing_ctrl_)->curr_foot_vel_des_), 
            VECT3, "rfoot_vel_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_left_swing_ctrl_)->curr_foot_vel_des_), 
            VECT3, "lfoot_vel_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_right_swing_ctrl_)->curr_foot_acc_des_), 
            VECT3, "rfoot_acc_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)config_left_swing_ctrl_)->curr_foot_acc_des_), 
            VECT3, "lfoot_acc_des", 3);

    printf("[Walking Config Test] Constructed\n");
}

WalkingConfigTest::~WalkingConfigTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void WalkingConfigTest::TestInitialization(){
    // Planner
    reversal_planner_->PlannerInitialization(MercuryConfigPath"PLANNER_velocity_reversal");
    // Yaml file name
    jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    config_body_fix_ctrl_->CtrlInitialization("CTRL_fix_config");
    // Transition
    right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
    right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
    left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
    left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
    // Swing
    config_right_swing_ctrl_->CtrlInitialization("CTRL_config_right_walking_swing");
    config_left_swing_ctrl_->CtrlInitialization("CTRL_config_left_walking_swing");
}

int WalkingConfigTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
#if (CONFIG_INITIAL_SWING_FOOT == 1)  
    if(phase == WkConfigPhase::lift_up) { next_phase = WkConfigPhase::double_contact_2; }
#endif
    //printf("next phase: %i\n", next_phase);
    dynacore::Vect3 next_local_frame_location;

    if(phase == WkConfigPhase::double_contact_1) {
        //if(phase == WkConfigPhase::right_swing_start_trans) {
        ++num_step_;
        //printf("%i th step:\n", num_step_);
        // printf("One swing done: Next Right Leg Swing\n");
        sp_->stance_foot_ = mercury_link::leftFoot;

        // Global Frame Update
        robot_sys_->getPos(mercury_link::leftFoot, next_local_frame_location);
        // when it start the left leg is already stance foot and 
        // global set by 0.15
        if(num_step_>1) { 
            sp_->global_pos_local_ += next_local_frame_location; 
            sp_->global_jjpos_local_ += sp_->jjpos_lfoot_pos_; 
        }
    }
    if(phase == WkConfigPhase::double_contact_2){
        //if(phase == WkConfigPhase::left_swing_start_trans){
        ++num_step_;
        // printf("%i th step:\n", num_step_);

        sp_->stance_foot_ = mercury_link::rightFoot;

        // Global Frame Update
        robot_sys_->getPos(mercury_link::rightFoot, next_local_frame_location);
        sp_->global_pos_local_ += next_local_frame_location;
        sp_->global_jjpos_local_ += sp_->jjpos_rfoot_pos_; 
    }
    // TEST: Using MoCap to estimate Global Location
    if( ( (phase == WkConfigPhase::double_contact_1) ||
                (phase == WkConfigPhase::double_contact_2) ) &&
            (num_step_>1) ){
        sp_->global_pos_local_[0] = 
            sp_->first_LED_x_ +  (next_local_frame_location[0] - sp_->Q_[0]);
        sp_->global_pos_local_[1] = 
            sp_->first_LED_y_ + (next_local_frame_location[1] - sp_->Q_[1]);
    }

    sp_->num_step_copy_ = num_step_;

    if(next_phase == WkConfigPhase::NUM_WALKING_PHASE) {
        return WkConfigPhase::double_contact_1;
    }
    else{ 
        return next_phase; 
    }
}

void WalkingConfigTest::_SettingParameter(){
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath"TEST_walking_config.yaml");

    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;

    // Swing Controller Selection
    handler.getString("swing_ctrl_type", tmp_str);
    if(tmp_str == "ConfigBodyFootPlanningCtrl"){
        config_right_swing_ctrl_ = 
            new ConfigBodyFootPlanningCtrl(robot_sys_, mercury_link::rightFoot, reversal_planner_);
        config_left_swing_ctrl_ = 
            new ConfigBodyFootPlanningCtrl(robot_sys_, mercury_link::leftFoot, reversal_planner_);
        ////////////////////////////////////////////////////////////////////////////////////////////
    }else{
        printf("[Warning] No matched planning controller\n");
        exit(0);
    }
    // Start Phase
    handler.getInteger("start_phase", phase_);

    //// Posture Setup
    // Initial JPos
    handler.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
    // Body Pitch
    handler.getValue("body_pitch", sp_->des_body_pitch_);
    // CoM Height
    handler.getValue("body_height", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((ConfigBodyCtrl*)config_body_fix_ctrl_)->setStanceHeight(tmp);

    ((TransitionConfigCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((TransitionConfigCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
    ((TransitionConfigCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((TransitionConfigCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setStanceHeight(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setStanceHeight(tmp);
    ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

    //// Timing Setup
    handler.getValue("jpos_initialization_time", tmp);
    ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
    handler.getValue("com_lifting_time", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

    // Stance Time
    handler.getValue("stance_time", tmp);
    ((ConfigBodyCtrl*)config_body_fix_ctrl_)->setStanceTime(tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->notifyStanceTime(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->notifyStanceTime(tmp);

    // Swing & prime Time
    handler.getValue("swing_time", tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setSwingTime(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setSwingTime(tmp);

    // Transition Time
    handler.getValue("st_transition_time", tmp);
    ((TransitionConfigCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((TransitionConfigCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
    ((TransitionConfigCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((TransitionConfigCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->notifyTransitionTime(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->notifyTransitionTime(tmp);
    //// Planner Setup
    handler.getValue("planning_frequency", tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setPlanningFrequency(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setPlanningFrequency(tmp);

    handler.getValue("double_stance_mix_ratio", tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setDoubleStanceRatio(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setDoubleStanceRatio(tmp);

    handler.getValue("transition_phase_mix_ratio", tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

    handler.getBoolean("contact_switch_check", b_tmp);
    ((SwingPlanningCtrl*)config_right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
    ((SwingPlanningCtrl*)config_left_swing_ctrl_)->setContactSwitchCheck(b_tmp);

    printf("[Walking Body Test] Complete to Setup Parameters\n");
}

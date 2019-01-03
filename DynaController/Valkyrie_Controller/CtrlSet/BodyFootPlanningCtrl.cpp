#include "BodyFootPlanningCtrl.hpp"
#include <Configuration.h>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>

#include <Valkyrie_Controller/TaskSet/LinkPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosSelectTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkGlobalSelectPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkOriTask.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/SelectedJPosTask.hpp>

#include <Valkyrie_Controller/ContactSet/SingleContact.hpp>

BodyFootPlanningCtrl::BodyFootPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    push_down_height_(0.),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint),
    des_jacc_(valkyrie::num_act_joint),
    waiting_time_limit_(0.02),
    Kp_(valkyrie::num_act_joint),
    Kd_(valkyrie::num_act_joint)
{
    des_jacc_.setZero();
    rfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::rightFoot);
    lfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::leftFoot);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    //lhand_pos_task_ = new LinkPosSelectTask(robot, valkyrie_link::leftPalm, 2);
    lhand_pos_task_ = new LinkGlobalSelectPosTask(robot, valkyrie_link::leftPalm, 1);
    lhand_ori_task_ = new LinkOriTask(robot, valkyrie_link::leftPalm);

    head_ori_task_ = new LinkOriTask(robot, valkyrie_link::head);

    selected_jidx_.push_back(valkyrie_joint::lowerNeckPitch);
    selected_jidx_.push_back(valkyrie_joint::neckYaw);
    selected_jidx_.push_back(valkyrie_joint::upperNeckPitch);

    head_joint_task_ = new SelectedJPosTask(selected_jidx_);

    //body_pos_task_ = new LinkPosTask(robot_sys_, valkyrie_link::pelvis);
    body_pos_task_ = new LinkPosSelectTask(robot_sys_, valkyrie_link::pelvis, 2);
    body_ori_task_ = new LinkOriTask(robot_sys_, valkyrie_link::pelvis);
    torso_ori_task_ = new LinkOriTask(robot_sys_, valkyrie_link::torso);

    //foot_pos_task_ = new LinkPosTask(robot_sys_, swing_foot, false);
    //foot_ori_task_ = new LinkOriTask(robot_sys_, swing_foot, false);
    foot_pos_task_ = new LinkPosTask(robot_sys_, swing_foot);
    foot_ori_task_ = new LinkOriTask(robot_sys_, swing_foot);
    total_joint_task_ = new JPosTask();

    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(valkyrie::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(valkyrie::num_act_joint, -500.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(valkyrie::num_act_joint, 500.);

    kin_wbc_contact_list_.clear();
    int rf_idx_offset(0);
    if(swing_foot == valkyrie_link::leftFoot) {
        rf_idx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SingleContact*)lfoot_contact_)->setMaxFz(0.0001); 
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    }
    else if(swing_foot == valkyrie_link::rightFoot) {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SingleContact*)rfoot_contact_)->setMaxFz(0.0001); 
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);


    // Create Minimum jerk objects
    for(size_t i = 0; i < 3; i++){
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
    printf("[Configuration BodyFootPlanning Controller] Constructed\n");
}

void BodyFootPlanningCtrl::OneStep(void* _cmd){   
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        ((Valkyrie_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Valkyrie_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Valkyrie_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyFootPlanningCtrl::_contact_setup(){
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyFootPlanningCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(valkyrie::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
    //dynacore::pretty_print(wblc_data_->Fr_, std::cout, "Fr");
}

void BodyFootPlanningCtrl::_task_setup(){
    // Body Pos
    double body_height_cmd;
    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_pos_[2];

    dynacore::Vector vel_des(3); vel_des.setZero();
    dynacore::Vector acc_des(3); acc_des.setZero();
    dynacore::Vect3 des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);
    
    // Body & Foot Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);
    dynacore::Vector ang_vel_des(3); ang_vel_des.setZero();
    dynacore::Vector ang_acc_des(3); ang_acc_des.setZero();

    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);
    torso_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);
    foot_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    _CheckPlanning();
    // Foot Pos Task Setup 
    _foot_pos_task_setup();

    // Full joint task
    dynacore::Vector jpos_des = sp_->jpos_ini_;
    dynacore::Vector zero(valkyrie::num_act_joint); zero.setZero();
    //total_joint_task_->UpdateTask(&(jpos_des), zero, zero);

    // Left Hand
    vel_des.setZero(); acc_des.setZero();
    ini_lhand_pos_[1] = 0.3;
    //ini_lhand_pos_[2] = 1.0;
    //lhand_pos_task_->UpdateTask(&(ini_lhand_pos_), vel_des, acc_des);

    dynacore::Quaternion des_cup_quat;
    rpy_des.setZero();
    rpy_des[2] = -M_PI/2.;
    dynacore::convert(rpy_des, des_cup_quat);
 
    ang_vel_des.setZero();
    ang_acc_des.setZero();
    lhand_ori_task_->UpdateTask(&(des_cup_quat), ang_vel_des, ang_acc_des);

    // Head
    dynacore::Quaternion des_head_quat;
    rpy_des.setZero();
    double amp(0.0);
    double omega(0.5);
    rpy_des[2] = amp * sin(omega * sp_->curr_time_);
    dynacore::convert(rpy_des, des_head_quat);

    robot_sys_->getOri(valkyrie_link::head, des_head_quat);
    //head_ori_task_->UpdateTask(&(des_head_quat), ang_vel_des, ang_acc_des);
    dynacore::Vector neck_pos(3);
    dynacore::Vector neck_vel(3); neck_vel.setZero();
    dynacore::Vector neck_acc(3); neck_acc.setZero();
    neck_pos[0] = sp_->jpos_ini_[valkyrie_joint::lowerNeckPitch - valkyrie::num_virtual];
    neck_pos[1] = sp_->jpos_ini_[valkyrie_joint::neckYaw - valkyrie::num_virtual];
    neck_pos[2] = sp_->jpos_ini_[valkyrie_joint::upperNeckPitch - valkyrie::num_virtual];
    double head_rot_amp(0.5);
    double head_rot_omega(1.5);
    neck_pos[1] += head_rot_amp * sin(head_rot_omega * sp_->curr_time_);

    head_joint_task_->UpdateTask(&neck_pos, neck_vel, neck_acc);


    // Task Update
    //task_list_.push_back(head_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(lhand_ori_task_);

    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);

    //task_list_.push_back(lhand_pos_task_);
    
    task_list_.push_back(foot_pos_task_);
   task_list_.push_back(foot_ori_task_);
    //task_list_.push_back(total_joint_task_);

    task_list_.push_back(head_joint_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, kin_wbc_contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
}

void BodyFootPlanningCtrl::_foot_pos_task_setup(){
    //_GetSinusoidalSwingTrajectory();
    _GetBsplineSwingTrajectory();

    double traj_time = state_machine_time_ - half_swing_time_;
    if(state_machine_time_ > half_swing_time_){
        double pos, vel, acc;
        for(int i(0); i<3; ++i){
            min_jerk_offset_[i]->getPos(traj_time, pos);  
            min_jerk_offset_[i]->getVel(traj_time, vel);
            min_jerk_offset_[i]->getAcc(traj_time, acc);

            curr_foot_pos_des_[i] += pos;
            curr_foot_vel_des_[i] += vel;
            curr_foot_acc_des_[i] += acc;
        }
    }
    if(state_machine_time_> end_time_){
        for(int i(0); i<foot_pos_task_->getDim(); ++i){
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] = 
            -push_down_height_ - 0.1*(state_machine_time_ - end_time_);
    }

    dynacore::Vector foot_vel_des(foot_pos_task_->getDim()); foot_vel_des.setZero();
    dynacore::Vector foot_acc_des(foot_pos_task_->getDim()); foot_acc_des.setZero();
    foot_vel_des = curr_foot_vel_des_;
    foot_acc_des = curr_foot_acc_des_;

    foot_pos_task_->UpdateTask(
            &(curr_foot_pos_des_), 
            foot_vel_des, 
            foot_acc_des);
}

void BodyFootPlanningCtrl::_CheckPlanning(){
    if( (state_machine_time_ > 0.5 * end_time_) && b_replanning_ && !b_replaned_) {
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);

        dynacore::Vect3 target_offset;
        // X, Y target is originally set by intial_traget_loc
        for(int i(0); i<2; ++i)
            target_offset[i] = target_loc[i] - initial_target_loc_[i];

        // Foot height (z) is set by the initial height
        target_offset[2] = 0.; //target_loc[2] - ini_foot_pos_[2];

        _SetMinJerkOffset(target_offset);
        b_replaned_ = true;
    }
}

void BodyFootPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);

    // TEST 
    for(int i(0); i<2; ++i){
        //com_pos[i] = sp_->Q_[i] + body_pt_offset_[i];
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        // com_pos[i] += body_pt_offset_[i];
        //com_vel[i] = sp_->Qdot_[i];

        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
    }
    //printf("planning com state: %f, %f, %f, %f\n",
    //        com_pos[0], com_pos[1],
    //        com_vel[0], com_vel[1]);

    OutputReversalPL pl_output;
    ParamReversalPL pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_;


    pl_param.des_loc = sp_->des_location_;
    pl_param.stance_foot_loc = sp_->global_pos_local_;

    if(swing_foot_ == valkyrie_link::leftFoot)
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;


    dynacore::Vect3 tmp_global_pos_local = sp_->global_pos_local_;

    planner_->getNextFootLocation(com_pos + tmp_global_pos_local,
            com_vel,
            target_loc,
            &pl_param, &pl_output);
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local_;

    target_loc[2] = initial_target_loc_[2];

    // TEST
    for(int i(0); i<2; ++i){
        target_loc[i] += foot_landing_offset_[i];
    }
    //dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}

void BodyFootPlanningCtrl::FirstVisit(){
    b_replaned_ = false;
    ini_config_ = sp_->Q_;
    robot_sys_->getPos(valkyrie_link::leftPalm, ini_lhand_pos_);
    robot_sys_->getPos(valkyrie_link::pelvis, ini_body_pos_);
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    initial_target_loc_[0] = sp_->Q_[0];
    initial_target_loc_[1] = sp_->Q_[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_;

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    dynacore::Vect3 foot_pos_offset; foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;
    _SetMinJerkOffset(foot_pos_offset);

    dynacore::Vect3 com_vel;
    robot_sys_->getCoMPosition(ini_com_pos_);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = ini_com_pos_[0];   
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

}

void BodyFootPlanningCtrl::_SetMinJerkOffset(const dynacore::Vect3 & offset){
    // Initialize Minimum Jerk Parameter Containers
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero(); 
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i]; 

        min_jerk_offset_[i]->setParams(
                init_params, final_params,
                0., half_swing_time_);  
    }
}

bool BodyFootPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > (end_time_ + waiting_time_limit_)){
        // printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
        //        state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == valkyrie_link::leftFoot && sp_->b_lfoot_contact_){
            contact_happen = true;
        }
        if(swing_foot_ == valkyrie_link::rightFoot && sp_->b_rfoot_contact_){
            contact_happen = true;
        }
        if(state_machine_time_ > end_time_ * 0.5 && contact_happen){
        //     printf("[Config Body Foot Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
        //             state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void BodyFootPlanningCtrl::CtrlInitialization(
        const std::string & setting_file_name){
    ini_base_height_ = sp_->Q_[valkyrie_joint::virtual_Z];
    std::vector<double> tmp_vec;

    // Setting Parameters
    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");
    handler.getValue("swing_height", swing_height_);
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("default_target_foot_location", tmp_vec);
    for(int i(0); i<3; ++i){
        default_target_loc_[i] = tmp_vec[i];
    }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<valkyrie::num_act_joint; ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<valkyrie::num_act_joint; ++i){
        Kd_[i] = tmp_vec[i];
    }

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }

    handler.getVector("foot_landing_offset", foot_landing_offset_);

    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
    //printf("[Body Foot JPos Planning Ctrl] Parameter Setup Completed\n");
}

BodyFootPlanningCtrl::~BodyFootPlanningCtrl(){
    delete wblc_;
    delete lfoot_contact_;
    delete rfoot_contact_;
}

void BodyFootPlanningCtrl::_GetBsplineSwingTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for(int i(0); i<3; ++i){
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
}
void BodyFootPlanningCtrl::_GetSinusoidalSwingTrajectory(){
    curr_foot_acc_des_.setZero();
    for (int i(0); i<2; ++i){
        curr_foot_pos_des_[i] = 
            dynacore::smooth_changing(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
        curr_foot_vel_des_[i] = 
            dynacore::smooth_changing_vel(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
        curr_foot_acc_des_[i] = 
            dynacore::smooth_changing_acc(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
    }
    // for Z (height)
    double amp(swing_height_/2.);
    double omega ( 2.*M_PI /end_time_ );

    curr_foot_pos_des_[2] = 
        ini_foot_pos_[2] + amp * (1-cos(omega * state_machine_time_));
    curr_foot_vel_des_[2] = 
        amp * omega * sin(omega * state_machine_time_);
    curr_foot_acc_des_[2] = 
        amp * omega * omega * cos(omega * state_machine_time_);
}

void BodyFootPlanningCtrl::_SetBspline(
        const dynacore::Vect3 & st_pos, 
        const dynacore::Vect3 & des_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    dynacore::Vect3 middle_pos;

    middle_pos = (st_pos + des_pos)/2.;
    middle_pos[2] = swing_height_;

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = 0.;
        init[i+6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = -0.5;
    fin[8] = 5.;
    foot_traj_.SetParam(init, fin, middle_pt, end_time_);

    delete [] *middle_pt;
    delete [] middle_pt;    
}



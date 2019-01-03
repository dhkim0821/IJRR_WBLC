#include "BodyFootPlanningCtrl.hpp"
#include <Configuration.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <DracoBip_Controller/TaskSet/BodyTask.hpp>
#include <DracoBip_Controller/TaskSet/FootTask.hpp>
#include <DracoBip_Controller/TaskSet/SelectedJointTask.hpp>

#include <DracoBip_Controller/ContactSet/SingleContact.hpp>
#include <DracoBip_Controller/ContactSet/FootNoYaw.hpp>
#include <DracoBip_Controller/ContactSet/FootLinear.hpp>

BodyFootPlanningCtrl::BodyFootPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    push_down_height_(0.),
    des_jpos_(dracobip::num_act_joint),
    des_jvel_(dracobip::num_act_joint),
    des_jacc_(dracobip::num_act_joint),
    Kp_(dracobip::num_act_joint),
    Kd_(dracobip::num_act_joint)
{
    des_jacc_.setZero();
    rfoot_contact_ = new ContactType(robot_sys_, dracobip_link::rAnkle);
    lfoot_contact_ = new ContactType(robot_sys_, dracobip_link::lAnkle);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    selected_jidx_.clear();
    selected_jidx_.push_back(dracobip_joint::rHipYaw);
    selected_jidx_.push_back(dracobip_joint::lHipYaw);

    selected_joint_task_ = new SelectedJointTask(selected_jidx_);

    std::vector<bool> act_list;
    act_list.resize(dracobip::num_qdot, true);
    for(int i(0); i<dracobip::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(dracobip::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(dracobip::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(dracobip::num_act_joint, 100.);

    kin_wbc_contact_list_.clear();
    int jidx_offset(0);
    if(swing_foot == dracobip_link::lAnkle) {
        jidx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] = 0.5;

        ((ContactType*)lfoot_contact_)->setMaxFz(0.0001); 
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    }
    else if(swing_foot == dracobip_link::rAnkle) {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] = 0.5;

        ((ContactType*)rfoot_contact_)->setMaxFz(0.0001); 
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    foot_task_ = new FootTask(robot_sys_, swing_foot);
    base_task_ = new BodyTask(robot_sys_);

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

    for(int i(0); i<dracobip::num_act_joint; ++i){
        ((DracoBip_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((DracoBip_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((DracoBip_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
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
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<dracobip::num_act_joint; ++i){
        A_rotor(i + dracobip::num_virtual, i + dracobip::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();

    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(dracobip::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd_ = wblc_data_->qddot_;

    //dynacore::pretty_print(wblc_data_->Fr_, std::cout, "Fr");
    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");

}

void BodyFootPlanningCtrl::_task_setup(){
    dynacore::Vector jpos_des(2); jpos_des.setZero();
    dynacore::Vector jvel_des(2); jvel_des.setZero();
    dynacore::Vector jacc_des(2); jacc_des.setZero();

    selected_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);

    /////// Body Posture Task Setup 
    // Orientation
    dynacore::Quaternion des_quat;
    dynacore::convert(0., 0., 0., des_quat); // yaw, pitch, roll
    dynacore::Vector pos_des(7); pos_des.setZero();
    dynacore::Vector vel_des(6); vel_des.setZero();
    dynacore::Vector acc_des(6); acc_des.setZero();

    pos_des[0] = des_quat.x();
    pos_des[1] = des_quat.y();
    pos_des[2] = des_quat.z();
    pos_des[3] = des_quat.w();

    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_body_height_;
    pos_des[4] = 0.;
    pos_des[5] = ini_body_pos_[1];
    pos_des[6] = base_height_cmd;

    base_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    /////// Foot Pos Task Setup 
    _CheckPlanning();
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
        for(int i(0); i<foot_task_->getDim(); ++i){
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] = 
            -push_down_height_ - 0.1*(state_machine_time_ - end_time_);
    }

    dynacore::Vector foot_pos_des(7);
    dynacore::Vector foot_vel_des(foot_task_->getDim()); foot_vel_des.setZero();
    dynacore::Vector foot_acc_des(foot_task_->getDim()); foot_acc_des.setZero();
    dynacore::convert(0., 0., 0., des_quat); // yaw, pitch, roll

    foot_pos_des[0] = des_quat.x();
    foot_pos_des[1] = des_quat.y();
    foot_pos_des[2] = des_quat.z();
    foot_pos_des[3] = des_quat.w();

    foot_pos_des.tail(3) = curr_foot_pos_des_;
    foot_vel_des.tail(3) = curr_foot_vel_des_;
    foot_acc_des.tail(3) = curr_foot_acc_des_;

    foot_task_->UpdateTask(
            &(foot_pos_des), 
            foot_vel_des, 
            foot_acc_des);

    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(base_task_);
    task_list_.push_back(foot_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, kin_wbc_contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
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
    printf("com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);


    // TEST 
    for(int i(0); i<2; ++i){
        com_pos[i] = sp_->Q_[i] + body_pt_offset_[i];
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        // com_pos[i] += body_pt_offset_[i];
        com_vel[i] = sp_->Qdot_[i];

        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
    }
    OutputReversalPL pl_output;
    ParamReversalPL pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_;


    pl_param.des_loc = sp_->des_location_;
    pl_param.stance_foot_loc = sp_->global_pos_local_;

    if(swing_foot_ == dracobip_link::lAnkle)
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
    printf("planning com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);

    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}

void BodyFootPlanningCtrl::FirstVisit(){
    b_replaned_ = false;
    ini_config_ = sp_->Q_;
    robot_sys_->getPos(dracobip_link::torso, ini_body_pos_);
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
    if(state_machine_time_ > (end_time_ )){
         printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
                state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == dracobip_link::lAnkle && sp_->b_lfoot_contact_){
            contact_happen = true;
        }
        if(swing_foot_ == dracobip_link::rAnkle && sp_->b_rfoot_contact_){
            contact_happen = true;
        }
        if(state_machine_time_ > end_time_ * 0.5 && contact_happen){
             printf("[Config Body Foot Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                     state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void BodyFootPlanningCtrl::CtrlInitialization(
        const std::string & setting_file_name){
    ini_base_height_ = sp_->Q_[dracobip_joint::virtual_Z];
    std::vector<double> tmp_vec;

    // Setting Parameters
    ParamHandler handler(DracoBipConfigPath + setting_file_name + ".yaml");
    handler.getValue("swing_height", swing_height_);
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("default_target_foot_location", tmp_vec);
    for(int i(0); i<3; ++i){
        default_target_loc_[i] = tmp_vec[i];
    }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<dracobip::num_act_joint; ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<dracobip::num_act_joint; ++i){
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



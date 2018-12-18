#include "JPosTargetCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* robot):Controller(robot),
    jpos_target_(mercury::num_act_joint),
    jpos_ini_(mercury::num_act_joint),
    end_time_(1000.0),
    b_external_initial_pos_set_(false),
    ctrl_start_time_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    Kp_(mercury::num_act_joint),
    Kd_(mercury::num_act_joint)
{
    fixed_body_contact_ = new FixedBodyContact(robot);
    dim_contact_ = fixed_body_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(mercury::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);

    wblc_data_->tau_min_ = dynacore::Vector::Constant(mercury::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(mercury::num_act_joint, 100.);

    sp_ = Mercury_StateProvider::getStateProvider();
}

JPosTargetCtrl::~JPosTargetCtrl(){
    delete fixed_body_contact_;
    delete wblc_;
    delete wblc_data_;
}

void JPosTargetCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void JPosTargetCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<mercury::num_act_joint; ++i){
        A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();

    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(mercury::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    //sp_->reaction_forces_ = wblc_data_->Fr_;

    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des jacc cmd");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
}


void JPosTargetCtrl::_task_setup(){
    for(int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = dynacore::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jvel_[i] = dynacore::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jacc_[i] = dynacore::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    }
}

void JPosTargetCtrl::_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosTargetCtrl::setInitialPosition(const std::vector<double> & jpos_ini){
    for(int i(0); i<mercury::num_act_joint; ++i){
        jpos_ini_[i] = jpos_ini[i];
    }
    b_external_initial_pos_set_ = true;
}
void JPosTargetCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
    if(!b_external_initial_pos_set_)
        jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    //printf("b_external setting:%d\n", b_external_initial_pos_set_);
    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(jpos_target_, std::cout, "jpos target");
}

void JPosTargetCtrl::LastVisit(){  }

bool JPosTargetCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosTargetCtrl::CtrlInitialization(const std::string & setting_file_name){
    if(!b_external_initial_pos_set_)
        jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i];

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}

void JPosTargetCtrl::setTargetPosition(const std::vector<double>& jpos){
    for(int i(0); i<mercury::num_act_joint; ++i){
        jpos_target_[i] = jpos[i];
         //printf("%i th jpos: %f\n", i, jpos[i]);
    }
}

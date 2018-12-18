#include "JPosCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    Kp_(mercury::num_act_joint),
    Kd_(mercury::num_act_joint),
    ctrl_start_time_(0.)
{
    set_jpos_.resize(mercury::num_act_joint, 0.);
    amp_.resize(mercury::num_act_joint, 0.);
    freq_.resize(mercury::num_act_joint, 0.);
    phase_.resize(mercury::num_act_joint, 0.);

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
    printf("[Joint Position Control] Constructed\n");
}

JPosCtrl::~JPosCtrl(){
    delete fixed_body_contact_;
    delete wblc_;
    delete wblc_data_;
}

void JPosCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(mercury::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

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

void JPosCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
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
    sp_->reaction_forces_ = wblc_data_->Fr_;
}

void JPosCtrl::_task_setup(){
    double ramp_value(1.);
    double ramp_duration(0.5);

    if(state_machine_time_ < ramp_duration)
        ramp_value = state_machine_time_/ramp_duration;

    if(trj_type_ == mercury_trj_type::sinusoidal){
        double omega;

        for(int i(0); i<mercury::num_act_joint; ++i){
            omega = 2. * M_PI * freq_[i];
            if(b_jpos_set_)
                des_jpos_[i] = set_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
            else
                des_jpos_[i] = jpos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);

            des_jvel_[i] = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
            des_jvel_[i] *= ramp_value;
            des_jacc_[i] = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
            des_jacc_[i] *= ramp_value;
        }
    } else if(trj_type_ == mercury_trj_type::ramp){
        double rising_time;
        double ini_jpos;

        for (int i(0); i<mercury::num_act_joint; ++i){
            rising_time = state_machine_time_ - start_time_[i];
            if(b_jpos_set_){
                ini_jpos = set_jpos_[i];
            }else {
                ini_jpos = jpos_ini_[i];
            }
            if(state_machine_time_ < start_time_[i]){
                des_jpos_[i] = ini_jpos;
            }else if(state_machine_time_<start_time_[i] + delta_time_[i]){
                des_jpos_[i] = ini_jpos + jpos_delta_[i]*rising_time/delta_time_[i];
            }else {
                des_jpos_[i] = ini_jpos + jpos_delta_[i];
            }
            des_jvel_[i] = 0.;
        }
    }
}

void JPosCtrl::_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
}

void JPosCtrl::LastVisit(){
}

bool JPosCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i]; 

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}

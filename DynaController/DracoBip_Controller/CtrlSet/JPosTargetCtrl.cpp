#include "JPosTargetCtrl.hpp"
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip_Controller/TaskSet/JPosTask.hpp>
#include <DracoBip_Controller/ContactSet/FixedBodyContact.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>
#include <DracoBip/DracoBip_Model.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* robot):Controller(robot),
    jpos_target_(dracobip::num_act_joint),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    des_jpos_(dracobip::num_act_joint),
    des_jvel_(dracobip::num_act_joint),
    des_jacc_(dracobip::num_act_joint),
    Kp_(dracobip::num_act_joint),
    Kd_(dracobip::num_act_joint)
{
    jpos_task_ = new JPosTask();
    fixed_body_contact_ = new FixedBodyContact(robot);
    dim_contact_=fixed_body_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(dracobip::num_qdot, true);
    for(int i(0); i<dracobip::num_virtual; ++i) act_list[i] = false;

    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(dracobip::num_qdot,100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_,1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    
    wblc_data_->tau_min_ = dynacore::Vector::Constant(dracobip::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(dracobip::num_act_joint, 100.); 

/*
    wblc_data_->cost_weight = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);

    wblc_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 0.1);
*/
    sp_ = DracoBip_StateProvider::getStateProvider();

    printf( "[ CTRL - JPos Target] Constructed \n" );
}

JPosTargetCtrl::~JPosTargetCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wblc_;
    delete wblc_data_;
}

void JPosTargetCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc(gamma);

    double ramp_period(0.5);
    double ramp(1.0);
    if(state_machine_time_ < ramp_period) ramp = state_machine_time_/ramp_period;

    for(int i(0); i<dracobip::num_act_joint; ++i){
        ((DracoBip_Command*)_cmd)->jtorque_cmd[i] = ramp * gamma[i];
        ((DracoBip_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((DracoBip_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void JPosTargetCtrl::_jpos_ctrl_wbdc(dynacore::Vector & gamma){
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<dracobip::num_act_joint; ++i){
        A_rotor(i + dracobip::num_virtual, i + dracobip::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();
    std::vector<Task*> dummy_task_list_;
    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(dracobip::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;
}


void JPosTargetCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(dracobip::num_act_joint); jacc_des.setZero();

    for(int i(0); i<dracobip::num_act_joint; ++i){
        des_jpos_[i] = dynacore::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jvel_[i] = dynacore::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jacc_[i] = dynacore::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    }

    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, des_jacc_);
    task_list_.push_back(jpos_task_);
}

void JPosTargetCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosTargetCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
    jpos_ini_ = sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint);
}

void JPosTargetCtrl::LastVisit(){
}

bool JPosTargetCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

void JPosTargetCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint);
    ParamHandler handler(DracoBipConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
}

void JPosTargetCtrl::setTargetPosition(const std::vector<double>& jpos){
    for(int i(0); i<dracobip::num_act_joint; ++i){
        jpos_target_[i] = jpos[i];
    }
}

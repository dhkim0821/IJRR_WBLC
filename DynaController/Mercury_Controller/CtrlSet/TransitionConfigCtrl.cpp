#include "TransitionConfigCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/BaseTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

TransitionConfigCtrl::TransitionConfigCtrl(RobotSystem* robot, 
        int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    Kp_(mercury::num_act_joint),
    Kd_(mercury::num_act_joint)
{
    rfoot_contact_ = new SingleContact(robot_sys_, mercury_link::rightFoot);
    lfoot_contact_ = new SingleContact(robot_sys_, mercury_link::leftFoot);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(mercury::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[2] = 0.01;
    wblc_data_->W_rf_[5] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(mercury::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(mercury::num_act_joint, 100.);


    base_task_ = new BaseTask(robot_sys_);
    sp_ = Mercury_StateProvider::getStateProvider();
    //printf("[Transition Controller] Constructed\n");
}

TransitionConfigCtrl::~TransitionConfigCtrl(){
    delete base_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void TransitionConfigCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    
    kin_wbc_->Ainv_ = Ainv_;
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

void TransitionConfigCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
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

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    //dynacore::pretty_print(gamma, std::cout, "gamma");

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;
}

void TransitionConfigCtrl::_task_setup(){
    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_base_height_;
 
    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    rpy_des[1] = sp_->des_body_pitch_;
    dynacore::convert(rpy_des, des_quat);
   
    dynacore::Vector pos_des(5); pos_des.setZero();
    dynacore::Vector vel_des(base_task_->getDim()); vel_des.setZero();
    dynacore::Vector acc_des(base_task_->getDim()); acc_des.setZero();

    pos_des[0] = base_height_cmd;
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();
    pos_des[4] = des_quat.w();

    base_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(base_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    if(b_increase_){
        if(moving_foot_ == mercury_link::rightFoot){
            int swing_jidx = mercury_joint::rightAbduction - mercury::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i<3; ++i){
                des_jpos_[i + swing_jidx] *= h; 
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
             }
        }
        else if(moving_foot_ == mercury_link::leftFoot){
            int swing_jidx = mercury_joint::leftAbduction - mercury::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i<3; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
            }
        }
    }
}

void TransitionConfigCtrl::_contact_setup(){
    double alpha = 0.5 * (1-cos(M_PI * state_machine_time_/end_time_));
    double upper_lim(100.);
    double rf_weight(100.);
    double rf_weight_z(100.);
    double foot_weight(1000.);

    if(b_increase_){
        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha)  + 1000. * alpha;
    } else {
        upper_lim = max_rf_z_ - alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (alpha) * 5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha) * 0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha)  + 1000. * (1. - alpha);
    }
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    if(moving_foot_ == mercury_link::leftFoot) {
        wblc_data_->W_rf_[3] = rf_weight;
        wblc_data_->W_rf_[4] = rf_weight;
        wblc_data_->W_rf_[5] = rf_weight_z;

        wblc_data_->W_xddot_[3] = foot_weight;
        wblc_data_->W_xddot_[4] = foot_weight;
        wblc_data_->W_xddot_[5] = foot_weight;

        ((SingleContact*)lfoot_contact_)->setMaxFz(upper_lim); 
    }
    else if(moving_foot_ == mercury_link::rightFoot) {
        wblc_data_->W_rf_[0] = rf_weight;
        wblc_data_->W_rf_[1] = rf_weight;
        wblc_data_->W_rf_[2] = rf_weight_z;

        wblc_data_->W_xddot_[0] = foot_weight;
        wblc_data_->W_xddot_[1] = foot_weight;
        wblc_data_->W_xddot_[2] = foot_weight;

        ((SingleContact*)rfoot_contact_)->setMaxFz(upper_lim); 
    }
}

void TransitionConfigCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ini_base_height_ = sp_->Q_[mercury_joint::virtual_Z];
    ctrl_start_time_ = sp_->curr_time_;
}

void TransitionConfigCtrl::LastVisit(){
    sp_->des_jpos_prev_ = des_jpos_;
    // printf("[Transition] End\n");
}

bool TransitionConfigCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void TransitionConfigCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i];

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}

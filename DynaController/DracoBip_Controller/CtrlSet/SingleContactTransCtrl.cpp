#include "SingleContactTransCtrl.hpp"
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip_Controller/TaskSet/BodyTask.hpp>
#include <DracoBip_Controller/TaskSet/SelectedJointTask.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <DracoBip_Controller/ContactSet/SingleContact.hpp>
#include <DracoBip_Controller/ContactSet/FootNoYaw.hpp>
#include <DracoBip_Controller/ContactSet/FootLinear.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(RobotSystem* robot, 
        int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(dracobip::num_act_joint),
    des_jvel_(dracobip::num_act_joint),
    des_jacc_(dracobip::num_act_joint),
    Kp_(dracobip::num_act_joint),
    Kd_(dracobip::num_act_joint)
{
    base_task_ = new BodyTask(robot);

    selected_jidx_.clear();
    selected_jidx_.push_back(dracobip_joint::rHipYaw);
    selected_jidx_.push_back(dracobip_joint::lHipYaw);

    selected_joint_task_ = new SelectedJointTask(selected_jidx_);
    rfoot_contact_ = new ContactType(robot_sys_, dracobip_link::rAnkle);
    lfoot_contact_ = new ContactType(robot_sys_, dracobip_link::lAnkle);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

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

    sp_ = DracoBip_StateProvider::getStateProvider();
    //printf("[Transition Controller] Constructed\n");
}

SingleContactTransCtrl::~SingleContactTransCtrl(){
    delete base_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void SingleContactTransCtrl::OneStep(void* _cmd){
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

void SingleContactTransCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
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

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;
}

void SingleContactTransCtrl::_task_setup(){
    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_base_height_;

    dynacore::Vector jpos_des(2); jpos_des.setZero();
    dynacore::Vector jvel_des(2); jvel_des.setZero();
    dynacore::Vector jacc_des(2); jacc_des.setZero();

    selected_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
  
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

    pos_des[4] = 0.;
    pos_des[5] = ini_base_pos_[1];
    pos_des[6] = base_height_cmd;


    base_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    
    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(base_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    // TEST
    if(b_increase_){
        if(moving_foot_ == dracobip_link::rAnkle){
            int swing_jidx = dracobip_joint::rHipYaw - dracobip::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < dracobip::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h; 
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
             }
        }
        else if(moving_foot_ == dracobip_link::lAnkle){
            int swing_jidx = dracobip_joint::lHipYaw - dracobip::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < dracobip::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
            }
        }
    }
}

void SingleContactTransCtrl::_contact_setup(){
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

    int jidx_offset(0);
    if(moving_foot_ == dracobip_link::lAnkle) {
        jidx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((ContactType*)lfoot_contact_)->setMaxFz(upper_lim); 
    }
    else if(moving_foot_ == dracobip_link::rAnkle) {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((ContactType*)rfoot_contact_)->setMaxFz(upper_lim); 
    }
}

void SingleContactTransCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ini_base_height_ = sp_->Q_[dracobip_joint::virtual_Z];
    ctrl_start_time_ = sp_->curr_time_;

    robot_sys_->getPos(dracobip_link::torso, ini_base_pos_);
}

void SingleContactTransCtrl::LastVisit(){
    sp_->des_jpos_prev_ = des_jpos_;
    // printf("[Transition] End\n");
}

bool SingleContactTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void SingleContactTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    ParamHandler handler(DracoBipConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i];

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}

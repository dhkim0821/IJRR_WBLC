#include "SingleContactTransCtrl.hpp"
#include <NAO_Controller/NAO_StateProvider.hpp>

#include <NAO_Controller/TaskSet/LinkPosTask.hpp>
#include <NAO_Controller/TaskSet/LinkHeightTask.hpp>
#include <NAO_Controller/TaskSet/LinkOriTask.hpp>
#include <NAO_Controller/TaskSet/JPosTask.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <NAO/NAO_Model.hpp>
#include <NAO_Controller/NAO_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <NAO_Controller/ContactSet/SingleContact.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(RobotSystem* robot, 
        int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(nao::num_act_joint),
    des_jvel_(nao::num_act_joint),
    des_jacc_(nao::num_act_joint),
    Kp_(nao::num_act_joint),
    Kd_(nao::num_act_joint)
{

    total_joint_task_ = new JPosTask();
    body_pos_task_ = new LinkPosTask(robot, nao_link::torso);
    body_ori_task_ = new LinkOriTask(robot, nao_link::torso);

    rfoot_contact_ = new SingleContact(robot_sys_, nao_link::r_ankle);
    lfoot_contact_ = new SingleContact(robot_sys_, nao_link::l_ankle);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(nao::num_qdot, true);
    for(int i(0); i<nao::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(nao::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(nao::num_act_joint, -500.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(nao::num_act_joint, 500.);

    sp_ = NAO_StateProvider::getStateProvider();
    //printf("[Transition Controller] Constructed\n");
}

SingleContactTransCtrl::~SingleContactTransCtrl(){
    delete total_joint_task_;
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
    
    for(int i(0); i<nao::num_act_joint; ++i){
        ((NAO_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((NAO_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((NAO_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void SingleContactTransCtrl::_compute_torque_wblc(dynacore::Vector & gamma){

    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(nao::num_virtual, nao::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(nao::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
}

void SingleContactTransCtrl::_task_setup(){
    des_jpos_ = sp_->jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    double body_height_cmd;
    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_pos_[2];

    dynacore::Vector vel_des(3); vel_des.setZero();
    dynacore::Vector acc_des(3); acc_des.setZero();
    dynacore::Vect3 des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);    

    dynacore::Vector ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    dynacore::Vector ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    // Joint
    dynacore::Vector jpos_des = sp_->jpos_ini_;
    dynacore::Vector jvel_des(nao::num_act_joint); jvel_des.setZero();
    dynacore::Vector jacc_des(nao::num_act_joint); jacc_des.setZero();
    total_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
 
    // Task List Update
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(total_joint_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    if(b_increase_){
        if(moving_foot_ == nao_link::r_ankle){
            int swing_jidx = nao_joint::r_leg_hpz - nao::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < nao::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h; 
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
             }
        }
        else if(moving_foot_ == nao_link::l_ankle){
            int swing_jidx = nao_joint::l_leg_hpz - nao::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < nao::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
            }
        }
    }

    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
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
    if(moving_foot_ == nao_link::l_ankle) {
        jidx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((SingleContact*)lfoot_contact_)->setMaxFz(upper_lim); 
    }
    else if(moving_foot_ == nao_link::r_ankle) {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((SingleContact*)rfoot_contact_)->setMaxFz(upper_lim); 
    }
}

void SingleContactTransCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
    robot_sys_->getPos(nao_link::torso, ini_body_pos_);
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
    ParamHandler handler(NAOConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i];

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}

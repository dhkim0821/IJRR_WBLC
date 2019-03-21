#include "BodyCtrl.hpp"
#include <NAO_Controller/NAO_StateProvider.hpp>
#include <NAO_Controller/NAO_DynaCtrl_Definition.h>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <NAO_Controller/ContactSet/SingleContact.hpp>
#include <NAO_Controller/TaskSet/LinkPosTask.hpp>
#include <NAO_Controller/TaskSet/LinkHeightTask.hpp>
#include <NAO_Controller/TaskSet/LinkOriTask.hpp>
#include <NAO_Controller/TaskSet/SelectedJPosTask.hpp>
#include <NAO_Controller/TaskSet/JPosTask.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>


BodyCtrl::BodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(nao::num_act_joint),
    des_jvel_(nao::num_act_joint),
    des_jacc_(nao::num_act_joint),
    Kp_(nao::num_act_joint),
    Kd_(nao::num_act_joint)
{
    total_joint_task_ = new JPosTask();
    body_pos_task_ = new LinkPosTask(robot, nao_link::torso);
    body_ori_task_ = new LinkOriTask(robot, nao_link::torso);

    std::vector<bool> act_list;
    act_list.resize(nao::num_qdot, true);
    for(int i(0); i<nao::num_virtual; ++i) act_list[i] = false;
    
    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SingleContact(robot_sys_, nao_link::r_ankle);
    lfoot_contact_ = new SingleContact(robot_sys_, nao_link::l_ankle);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = dynacore::Vector::Constant(nao::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;
 
    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(nao::num_act_joint, -500.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(nao::num_act_joint, 500.);

    sp_ = NAO_StateProvider::getStateProvider();

    printf("[Config Body Control] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
    delete total_joint_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma = dynacore::Vector::Zero(nao::num_act_joint);
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

void BodyCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    // WBLC
    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(nao::num_virtual, nao::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(nao::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);
}

void BodyCtrl::_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    double body_height_cmd;
    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_pos_[2];
    //dynacore::pretty_print(ini_body_pos_, std::cout, "ini body pos");
    dynacore::Vector vel_des(3); vel_des.setZero();
    dynacore::Vector acc_des(3); acc_des.setZero();
    dynacore::Vect3 des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    dynacore::Quaternion des_quat;
    dynacore::convert(0., 0., 0., des_quat);

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

    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
}

void BodyCtrl::_contact_setup(){
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(nao::num_virtual, nao::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    robot_sys_->getPos(nao_link::torso, ini_body_pos_);
}

void BodyCtrl::LastVisit(){
}

bool BodyCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(nao::num_virtual, nao::num_act_joint);

    ParamHandler handler(NAOConfigPath + setting_file_name + ".yaml");

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

#include "BodyCtrl.hpp"
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip_Controller/TaskSet/BodyTask.hpp>
#include <DracoBip_Controller/TaskSet/SelectedJointTask.hpp>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

#include <DracoBip_Controller/ContactSet/SingleContact.hpp>
#include <DracoBip_Controller/ContactSet/FootNoYaw.hpp>
#include <DracoBip_Controller/ContactSet/FootLinear.hpp>
#include <DracoBip_Controller/ContactSet/FootLinear.hpp>

BodyCtrl::BodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(dracobip::num_act_joint),
    des_jvel_(dracobip::num_act_joint),
    des_jacc_(dracobip::num_act_joint),
    Kp_(dracobip::num_act_joint),
    Kd_(dracobip::num_act_joint)
{
    selected_jidx_.clear();
    selected_jidx_.push_back(dracobip_joint::rHipYaw);
    selected_jidx_.push_back(dracobip_joint::lHipYaw);

    selected_joint_task_ = new SelectedJointTask(selected_jidx_);
    body_task_ = new BodyTask(robot);

    std::vector<bool> act_list;
    act_list.resize(dracobip::num_qdot, true);
    for(int i(0); i<dracobip::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new ContactType(robot_sys_, dracobip_link::rAnkle);
    lfoot_contact_ = new ContactType(robot_sys_, dracobip_link::lAnkle);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = dynacore::Vector::Constant(dracobip::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(dracobip::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(dracobip::num_act_joint, 100.);

    sp_ = DracoBip_StateProvider::getStateProvider();

    printf("[Body Control] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
    delete body_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma = dynacore::Vector::Zero(dracobip::num_act_joint);
    _double_contact_setup();
    _body_task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<dracobip::num_act_joint; ++i){
        ((DracoBip_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((DracoBip_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((DracoBip_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    // WBLC
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
}

void BodyCtrl::_body_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    dynacore::Vector jpos_des(2); jpos_des.setZero();
    dynacore::Vector jvel_des(2); jvel_des.setZero();
    dynacore::Vector jacc_des(2); jacc_des.setZero();

    selected_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
 
    // Calculate IK for a desired height and orientation.
    double body_height_cmd;

    // Set Desired Orientation
    dynacore::Quaternion des_quat;
    dynacore::convert(0., 0., 0., des_quat); // yaw, pitch, roll

    dynacore::Vector pos_des(7); pos_des.setZero();
    dynacore::Vector vel_des(6); vel_des.setZero();
    dynacore::Vector acc_des(6); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;

    // Orientation
    pos_des[0] = des_quat.x();
    pos_des[1] = des_quat.y();
    pos_des[2] = des_quat.z();
    pos_des[3] = des_quat.w();
    // Position
    //pos_des.tail(3) = ini_body_pos_;
    pos_des[4] = 0.;
    pos_des[5] = ini_body_pos_[1];
    pos_des[6] = body_height_cmd;

    double amp(0.0);
    double omega(0.5 * 2. * M_PI);

    pos_des[6] += amp * sin(omega * state_machine_time_);
    vel_des[5] = amp * omega * cos(omega * state_machine_time_);

    body_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    
    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(body_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
    //
    //dynacore::Vect3 com_pos;
    //robot_sys_->getCoMPosition(com_pos);
    //dynacore::pretty_print(com_pos, std::cout, "com_pos");
}

void BodyCtrl::_double_contact_setup(){
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;

    robot_sys_->getPos(dracobip_link::torso, ini_body_pos_);
}

void BodyCtrl::LastVisit(){  }

bool BodyCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyCtrl::CtrlInitialization(const std::string & setting_file_name){
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

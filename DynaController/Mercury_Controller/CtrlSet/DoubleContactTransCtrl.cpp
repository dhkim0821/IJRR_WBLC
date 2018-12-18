#include "DoubleContactTransCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/BaseTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

DoubleContactTransCtrl::DoubleContactTransCtrl(RobotSystem* robot):
    Controller(robot),
    b_set_height_target_(false),
    end_time_(100.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    Kp_(mercury::num_act_joint),
    Kd_(mercury::num_act_joint)
{
    base_task_ = new BaseTask(robot);
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

    wblc_data_->tau_min_ = dynacore::Vector::Constant(mercury::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(mercury::num_act_joint, 100.);
    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Contact Transition Body Ctrl] Constructed\n");
}

DoubleContactTransCtrl::~DoubleContactTransCtrl(){
    delete base_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;
    delete wblc_;
    delete kin_wbc_;
    delete wblc_data_;
}

void DoubleContactTransCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    kin_wbc_->Ainv_ = Ainv_;
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

void DoubleContactTransCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
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

void DoubleContactTransCtrl::_task_setup(){
    des_jpos_ = ini_jpos_;
    des_jvel_.setZero();
    des_jacc_.setZero();
     // Calculate IK for a desired height and orientation.
    double base_height_cmd;

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();

    rpy_des[1] = sp_->des_body_pitch_;
    dynacore::convert(rpy_des, des_quat);    

    dynacore::Vector pos_des(5); pos_des.setZero();
    dynacore::Vector vel_des(base_task_->getDim()); vel_des.setZero();
    dynacore::Vector acc_des(base_task_->getDim()); acc_des.setZero();

    if(b_set_height_target_){
        base_height_cmd = 
            dynacore::smooth_changing(ini_base_height_, des_base_height_, 
                    end_time_, state_machine_time_);
    }else{
        printf("[Warning] The body height is not specified\n");
    }
    
    pos_des[0] = base_height_cmd;
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();
    pos_des[4] = des_quat.w();

    base_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(base_task_);
    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
}

void DoubleContactTransCtrl::_contact_setup(){
    ((SingleContact*)rfoot_contact_)->setMaxFz( 
        min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );
    ((SingleContact*)lfoot_contact_)->setMaxFz( 
        min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );

    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void DoubleContactTransCtrl::FirstVisit(){
    ini_base_height_ = sp_->Q_[mercury_joint::virtual_Z];
    ctrl_start_time_ = sp_->curr_time_;
}

void DoubleContactTransCtrl::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

bool DoubleContactTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void DoubleContactTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_jpos_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);
    // Feedback Gain
    std::vector<double> tmp_vec;
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
}

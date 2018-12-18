#include "DoubleContactTransCtrl.hpp"
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkOriTask.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>
#include <Valkyrie_Controller/ContactSet/SingleContact.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>


DoubleContactTransCtrl::DoubleContactTransCtrl(RobotSystem* robot):
    Controller(robot),
    b_set_height_target_(false),
    end_time_(100.),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint),
    des_jacc_(valkyrie::num_act_joint),
    Kp_(valkyrie::num_act_joint),
    Kd_(valkyrie::num_act_joint)
{
    total_joint_task_ = new JPosTask();
    //body_pos_task_ = new LinkHeightTask(robot, valkyrie_link::pelvis);
    body_pos_task_ = new LinkPosTask(robot, valkyrie_link::pelvis);
    body_ori_task_ = new LinkOriTask(robot, valkyrie_link::pelvis);
    torso_ori_task_ = new LinkOriTask(robot, valkyrie_link::torso);

    rfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::rightFoot);
    lfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::leftFoot);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(valkyrie::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    wblc_data_->tau_min_ = dynacore::Vector::Constant(valkyrie::num_act_joint, -200.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(valkyrie::num_act_joint, 200.);
    sp_ = Valkyrie_StateProvider::getStateProvider();

    printf("[Contact Transition Body Ctrl] Constructed\n");
}

DoubleContactTransCtrl::~DoubleContactTransCtrl(){
    delete total_joint_task_;
    delete body_pos_task_;
    delete body_ori_task_;

    delete wblc_;
    delete wblc_data_;
    delete kin_wbc_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void DoubleContactTransCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        ((Valkyrie_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Valkyrie_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Valkyrie_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void DoubleContactTransCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(valkyrie::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);
}

void DoubleContactTransCtrl::_task_setup(){
    des_jpos_ = ini_jpos_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    if(!b_set_height_target_){
        printf("Target Height is not specified\n"); 
        exit(0);
    }
    double body_height_cmd = 
            dynacore::smooth_changing(ini_body_pos_[2], target_body_height_, 
                    end_time_, state_machine_time_);
    dynacore::Vector vel_des(body_pos_task_->getDim()); vel_des.setZero();
    dynacore::Vector acc_des(body_pos_task_->getDim()); acc_des.setZero();
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
    torso_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    // Joint
    dynacore::Vector jpos_des = sp_->jpos_ini_;
    dynacore::Vector jvel_des(valkyrie::num_act_joint); jvel_des.setZero();
    dynacore::Vector jacc_des(valkyrie::num_act_joint); jacc_des.setZero();
    total_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
 
    // Task List Update
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(total_joint_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
}

void DoubleContactTransCtrl::_contact_setup(){
    //((SingleContact*)rfoot_contact_)->setMaxFz( 
        //min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );
    //((SingleContact*)lfoot_contact_)->setMaxFz( 
        //min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );

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
    ctrl_start_time_ = sp_->curr_time_;
    robot_sys_->getPos(valkyrie_link::pelvis, ini_body_pos_);
}

void DoubleContactTransCtrl::LastVisit(){
     printf("[ContactTransBody] End\n");
     //exit(0);
}

bool DoubleContactTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void DoubleContactTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_jpos_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");
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

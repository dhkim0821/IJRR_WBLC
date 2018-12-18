#include "BodyCtrl.hpp"
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaCtrl_Definition.h>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <Valkyrie_Controller/ContactSet/SingleContact.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosSelectTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkGlobalSelectPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkOriTask.hpp>
//#include <Valkyrie_Controller/TaskSet/LinkHeightTask.hpp>
#include <Valkyrie_Controller/TaskSet/SelectedJPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>


BodyCtrl::BodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint),
    des_jacc_(valkyrie::num_act_joint),
    Kp_(valkyrie::num_act_joint),
    Kd_(valkyrie::num_act_joint)
{
    total_joint_task_ = new JPosTask();

    selected_jidx_.push_back(valkyrie_joint::lowerNeckPitch);
    selected_jidx_.push_back(valkyrie_joint::neckYaw);
    selected_jidx_.push_back(valkyrie_joint::upperNeckPitch);

    head_joint_task_ = new SelectedJPosTask(selected_jidx_);

    //body_pos_task_ = new LinkHeightTask(robot, valkyrie_link::pelvis);
    //body_pos_task_ = new LinkPosTask(robot, valkyrie_link::pelvis);
    body_pos_task_ = new LinkPosSelectTask(robot_sys_, valkyrie_link::pelvis, 2);
    
    head_ori_task_ = new LinkOriTask(robot_sys_, valkyrie_link::head);

    //lhand_pos_task_ = new LinkPosSelectTask(robot, valkyrie_link::leftPalm, 2);
    lhand_pos_task_ = new LinkGlobalSelectPosTask(robot, valkyrie_link::leftPalm, 1);
    lhand_ori_task_ = new LinkOriTask(robot, valkyrie_link::leftPalm);

    body_ori_task_ = new LinkOriTask(robot, valkyrie_link::pelvis);
    torso_ori_task_ = new LinkOriTask(robot, valkyrie_link::torso);

    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;
    
    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::rightFoot);
    lfoot_contact_ = new SingleContact(robot_sys_, valkyrie_link::leftFoot);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = dynacore::Vector::Constant(valkyrie::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

 
    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(valkyrie::num_act_joint, -500.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(valkyrie::num_act_joint, 500.);

    sp_ = Valkyrie_StateProvider::getStateProvider();

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

    dynacore::Vector gamma = dynacore::Vector::Zero(valkyrie::num_act_joint);
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

void BodyCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    // WBLC
    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(valkyrie::num_act_joint));

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
    else body_height_cmd = ini_body_height_;
    dynacore::Vector vel_des(3); vel_des.setZero();
    dynacore::Vector acc_des(3); acc_des.setZero();
    dynacore::Vect3 des_pos = ini_body_pos_;

    double amp(0.0);
    double omega(0.5 * 2. * M_PI);
    des_pos[2] = body_height_cmd + amp * sin(omega * state_machine_time_);
    vel_des[2] = amp * omega * cos(omega * state_machine_time_);
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
    //total_joint_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);


    // Left Hand
    vel_des.setZero(); acc_des.setZero();
    ini_lhand_pos_[1] = 0.3;
    //ini_lhand_pos_[2] = 1.0;
    //lhand_pos_task_->UpdateTask(&(ini_lhand_pos_), vel_des, acc_des);

    dynacore::Quaternion des_cup_quat;
    rpy_des.setZero();
    rpy_des[2] = -M_PI/2.;
    dynacore::convert(rpy_des, des_cup_quat);
 
    ang_vel_des.setZero();
    ang_acc_des.setZero();
    lhand_ori_task_->UpdateTask(&(des_cup_quat), ang_vel_des, ang_acc_des);

    // Head
    //dynacore::Quaternion des_head_quat;
    //dynacore::Quaternion head_ori_ini;
    //head_ori_ini.w() = -0.9978;
    //head_ori_ini.x() = 0.;
    //head_ori_ini.y() = 0.0654;
    //head_ori_ini.z() = 0.;
    //rpy_des.setZero();
   //rpy_des[2] = head_rot_amp * sin(head_rot_omega * sp_->curr_time_);
    //dynacore::convert(rpy_des, des_head_quat);
    //dynacore::pretty_print(des_head_quat, std::cout, "vertical head ori"); 
    //des_head_quat = dynacore::QuatMultiply(des_head_quat, head_ori_ini);
    
    //robot_sys_->getOri(valkyrie_link::head, des_head_quat);
    //dynacore::pretty_print(des_head_quat, std::cout, "des head ori"); 
    //head_ori_task_->UpdateTask(&(des_head_quat), ang_vel_des, ang_acc_des);

    dynacore::Vector neck_pos(3);
    dynacore::Vector neck_vel(3); neck_vel.setZero();
    dynacore::Vector neck_acc(3); neck_acc.setZero();
    neck_pos[0] = sp_->jpos_ini_[valkyrie_joint::lowerNeckPitch - valkyrie::num_virtual];
    neck_pos[1] = sp_->jpos_ini_[valkyrie_joint::neckYaw - valkyrie::num_virtual];
    neck_pos[2] = sp_->jpos_ini_[valkyrie_joint::upperNeckPitch - valkyrie::num_virtual];

    double head_rot_amp(0.5);
    double head_rot_omega(1.5);
    neck_pos[1] += head_rot_amp * sin(head_rot_omega * sp_->curr_time_);
    head_joint_task_->UpdateTask(&neck_pos, neck_vel, neck_acc);

    // Task List Update
    //task_list_.push_back(head_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(lhand_ori_task_);

    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);

    task_list_.push_back(head_joint_task_);
    //task_list_.push_back(lhand_pos_task_);

   //task_list_.push_back(total_joint_task_);

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
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    robot_sys_->getPos(valkyrie_link::pelvis, ini_body_pos_);
    robot_sys_->getPos(valkyrie_link::leftPalm, ini_lhand_pos_);
    robot_sys_->getOri(valkyrie_link::leftPalm, ini_lhand_ori_);
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
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);

    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");

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

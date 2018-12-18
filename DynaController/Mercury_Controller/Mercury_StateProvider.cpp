#include "Mercury_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>
#include "MoCapManager.hpp"
#include <Mercury/Mercury_Model.hpp>

Mercury_StateProvider* Mercury_StateProvider::getStateProvider(){
    static Mercury_StateProvider state_provider_;
    return &state_provider_;
}

Mercury_StateProvider::Mercury_StateProvider(): initialized_(false),
    des_body_pitch_(0.),
    mjpos_(mercury::num_act_joint),
    stance_foot_(mercury_link::leftFoot),
    Q_(mercury::num_q),
    Qdot_(mercury::num_qdot),
    reaction_forces_(6),
    qddot_cmd_(mercury::num_qdot),
    jjpos_config_(mercury::num_q),
    jjvel_qdot_(mercury::num_qdot),
    rotor_inertia_(mercury::num_act_joint),
    b_rfoot_contact_(0),
    b_lfoot_contact_(0),
    estimated_com_state_(4),
    com_state_imu_(6),
    num_step_copy_(0),
    led_kin_data_(3*NUM_MARKERS),
    des_jpos_prev_(mercury::num_act_joint),
    first_LED_x_(0.),
    first_LED_y_(0.)
{
    mjpos_.setZero();
    des_jpos_prev_.setZero();

    jjpos_robot_sys_ = new Mercury_Model();
    jjpos_rfoot_pos_.setZero();
    jjpos_lfoot_pos_.setZero();

    est_CoM_vel_.setZero();
    est_mocap_body_pos_.setZero();
    est_mocap_body_vel_.setZero();

    Q_.setZero();
    Q_[mercury::num_qdot] = 1.;
    Qdot_.setZero();
    reaction_forces_.setZero();
    qddot_cmd_.setZero();
    global_pos_local_.setZero();
    global_jjpos_local_.setZero();
    des_location_.setZero();
    estimated_com_state_.setZero();

    rotor_inertia_.setZero();

    CoM_pos_.setZero();
    CoM_vel_.setZero();
    com_pos_des_.setZero();
    com_vel_des_.setZero();

    body_ori_rpy_.setZero();
    average_vel_.setZero();

    Rfoot_pos_.setZero();
    Rfoot_vel_.setZero();
    Lfoot_pos_.setZero();
    Lfoot_vel_.setZero();

    com_state_imu_.setZero();

    phase_copy_= 0;

    DataManager* data_manager = DataManager::GetDataManager();

    led_kin_data_.setZero();
    //data_manager->RegisterData(&led_kin_data_, DYN_VEC, "LED_Kin_Pos", 3*NUM_MARKERS);
    data_manager->RegisterData(&average_vel_, VECT2, "average_vel", 2);

    data_manager->RegisterData(&curr_time_, DOUBLE, "time");
    data_manager->RegisterData(&mjpos_, DYN_VEC, "mjpos", mercury::num_act_joint);
    data_manager->RegisterData(&Q_, DYN_VEC, "config", mercury::num_q);
    data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", mercury::num_qdot);
    data_manager->RegisterData(&reaction_forces_, DYN_VEC, "reaction_force", 6);
    data_manager->RegisterData(&qddot_cmd_, DYN_VEC, "qddot_cmd", mercury::num_qdot);

    data_manager->RegisterData(&jjpos_config_, DYN_VEC, "jjpos_config", mercury::num_q);
    data_manager->RegisterData(&jjvel_qdot_, DYN_VEC, "jjvel_qdot", mercury::num_qdot);
    data_manager->RegisterData(&jjpos_rfoot_pos_, VECT3, "jjpos_rfoot_pos", 3);
    data_manager->RegisterData(&jjpos_lfoot_pos_, VECT3, "jjpos_lfoot_pos", 3);

    data_manager->RegisterData(&Rfoot_pos_, VECT3, "rfoot_pos", 3);
    data_manager->RegisterData(&Rfoot_vel_, VECT3, "rfoot_vel", 3);
    data_manager->RegisterData(&Lfoot_pos_, VECT3, "lfoot_pos", 3);
    data_manager->RegisterData(&Lfoot_vel_, VECT3, "lfoot_vel", 3);
    //data_manager->RegisterData(&foot_pos_des_, VECT3, "foot_pos_des", 3);
    //data_manager->RegisterData(&foot_vel_des_, VECT3, "foot_vel_des", 3);

    data_manager->RegisterData(&CoM_pos_, VECT3, "com_pos", 3);
    data_manager->RegisterData(&CoM_vel_, VECT3, "com_vel", 3);
    data_manager->RegisterData(&est_CoM_vel_, VECT2, "est_com_vel", 2);
    data_manager->RegisterData(&est_mocap_body_vel_, VECT2, "est_mocap_body_vel", 2);
    data_manager->RegisterData(&est_mocap_body_pos_, VECT3, "est_mocap_body_pos", 3);
        data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);
    data_manager->RegisterData(&global_jjpos_local_, VECT3, "global_jjpos_local", 3);


    // data_manager->RegisterData(&imu_acc_inc_, VECT3, "imu_acc_inc", 3);
    data_manager->RegisterData(&imu_acc_, VECT3, "imu_acc", 3);
    // data_manager->RegisterData(&imu_ang_vel_, VECT3, "imu_ang_vel", 3);
    //data_manager->RegisterData(&com_state_imu_, DYN_VEC, "com_state_imu", 6);

        //data_manager->RegisterData(&rotor_inertia_, DYN_VEC, "rotor_inertia", mercury::num_act_joint);

    // Foot Contact 
    data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);
    data_manager->RegisterData(&estimated_com_state_, DYN_VEC, "estimated_com_state", 4);

    // Simulation Ground Truth
    sim_imu_pos.setZero();
    sim_imu_vel.setZero();    
    // data_manager->RegisterData(&sim_imu_pos, VECT3, "sim_imu_pos", 3);
    // data_manager->RegisterData(&sim_imu_vel, VECT3, "sim_imu_vel", 3);
}


void Mercury_StateProvider::SaveCurrentData(Mercury_SensorData* data, 
        RobotSystem* ctrl_robot){

    dynacore::Vect3 pos;
    int led_idx = mercury_link::LED_BODY_0;
    for(int i(0); i<NUM_MARKERS; ++i){
        ctrl_robot->getPos(mercury_link::LED_BODY_0 + i, pos);
        for (int j(0); j<3; ++j)  led_kin_data_[3*i + j] = pos[j];
    }

    for(int i(0); i<3; ++i){
        imu_acc_inc_[i] = data->imu_inc[i];
        imu_acc_[i] = data->imu_acc[i];
        imu_ang_vel_[i] = data->imu_ang_vel[i];
    }

    double yaw, pitch, roll;
    dynacore::Quaternion body_ori;
    body_ori.w() = Q_[mercury_joint::virtual_Rw];
    body_ori.x() = Q_[mercury_joint::virtual_Rx];
    body_ori.y() = Q_[mercury_joint::virtual_Ry];
    body_ori.z() = Q_[mercury_joint::virtual_Rz];
    dynacore::convert(body_ori, yaw, pitch, roll);
    body_ori_rpy_[0] = roll;
    body_ori_rpy_[1] = pitch;
    body_ori_rpy_[2] = yaw;

    // Foot pos
    ctrl_robot->getPos(mercury_link::rightFoot, Rfoot_pos_);
    ctrl_robot->getLinearVel(mercury_link::rightFoot, Rfoot_vel_);
    ctrl_robot->getPos(mercury_link::leftFoot, Lfoot_pos_);
    ctrl_robot->getLinearVel(mercury_link::leftFoot, Lfoot_vel_);

    jjpos_robot_sys_->getPos(mercury_link::rightFoot, jjpos_rfoot_pos_);
    jjpos_robot_sys_->getPos(mercury_link::leftFoot, jjpos_lfoot_pos_);
}

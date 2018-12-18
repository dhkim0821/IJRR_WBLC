#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Atlas/Atlas_Definition.h>

BasicAccumulation::BasicAccumulation():filtered_acc_(3){
    global_ori_.w() = 1.;
    global_ori_.x() = 0.;
    global_ori_.y() = 0.;
    global_ori_.z() = 0.;

    cutoff_freq_ = 2.0* M_PI *1.0; // 1Hz // (2*pi*frequency) rads/s 
    for(int i(0); i<3; ++i){
        filtered_acc_[i] = new digital_lp_filter(cutoff_freq_, atlas::servo_rate);
    }
    global_ang_vel_.setZero();
}
void BasicAccumulation::EstimatorInitialization(      
        const std::vector<double> & acc,
        const std::vector<double> & ang_vel){

    for(int i(0); i<3; ++i){
        global_ang_vel_[i] = ang_vel[i];
        filtered_acc_[i]->input(acc[i]);
    }
    _InitIMUOrientationEstimateFromGravity();
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
        const std::vector<double> & ang_vel){

    // Convert body omega into a delta quaternion ------------------------------
    dynacore::Vect3 body_omega; body_omega.setZero();
    for(size_t i = 0; i < 3; i++){
        body_omega[i] = ang_vel[i];
    }
    dynacore::Quaternion delta_quat_body;
    dynacore::convert(body_omega*atlas::servo_rate, delta_quat_body);

    dynacore::Matrix R_global_to_imu = global_ori_.normalized().toRotationMatrix();
    global_ang_vel_ = R_global_to_imu * body_omega;

    // Perform orientation update via integration
    global_ori_ = dynacore::QuatMultiply(global_ori_, delta_quat_body); 
}


void BasicAccumulation::_InitIMUOrientationEstimateFromGravity(){
    dynacore::Vect3 g_B; g_B.setZero();
    for(int i(0); i<3; ++i){
        // We expect a negative number if gravity is pointing opposite of 
        // the IMU direction
        g_B[i] = -filtered_acc_[i]->output();
    }
    // Test Vector  ////////////////////////
    //g_B[0] = 0.1; g_B[1] = 0.4; g_B[2] = 0.5;
    //g_B[0] = 0.0; g_B[1] = 0.0; g_B[2] = 1.0;
    //dynacore::Quaternion rot_quat;
    //dynacore::convert(1.0, -0.5, 0.1, rot_quat);
    //Eigen::Matrix3d rot_mt(rot_quat);
    //g_B = rot_mt * g_B;
    //////////////////////////////////////////////
    g_B.normalize();

    double theta_pitch = atan2(g_B[0], g_B[2]);

    dynacore::Vect3 g_B_xz = g_B;
    g_B_xz[1]=0.;
    g_B_xz.normalize();

    double inner = g_B_xz.transpose() * g_B;
    double theta_roll = acos(inner);
    if(g_B[1]>0) theta_roll *= (-1.);

    dynacore::Quaternion quat_pitch, quat_roll;
    dynacore::convert(0., theta_pitch, 0., quat_pitch);
    dynacore::convert(0., 0., theta_roll, quat_roll);
    dynacore::Quaternion local2Glob = dynacore::QuatMultiply(quat_pitch, quat_roll);
    global_ori_ = local2Glob.inverse();

    ///////////////////////////////////////////////////////////////////
    //Eigen::Matrix3d Global_ori(global_ori_);
    //dynacore::Vect3 check_vec = Global_ori * g_B;
    //dynacore::pretty_print(check_vec, std::cout, "check vec");
    //dynacore::pretty_print(g_B, std::cout, "g_B");
    //printf("pitch, roll: %f, %f\n", theta_pitch, theta_roll);
}

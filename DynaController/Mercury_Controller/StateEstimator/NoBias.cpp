#include "NoBias.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Mercury/Mercury_Definition.h>

NoBias::NoBias():OriEstimator(),
                             x_(DIM_STATE_NO_BIAS - 3),
                             x_pred_(DIM_STATE_NO_BIAS - 3)
{
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  ori_pred_.w() = 1.;
  ori_pred_.x() = 0.;
  ori_pred_.y() = 0.;
  ori_pred_.z() = 0.;

  P_.setIdentity();
  P_pred_.setIdentity();

  x_.setZero();
  x_pred_.setZero();

  Q_.setIdentity();
  R_.setIdentity();
  // Velocity
  // Q_.block<3,3>(0,0) *= 1.0;
  // Q_.block<3,3>(3,3) *= 10.0;
  // Q_.block<3,3>(6,6) *= 0.1;
  // R_*=1000.0;
}


NoBias::~NoBias(){}

void NoBias::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                     const std::vector<double> & acc,
                                         const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i)  global_ang_vel_[i] = ang_vel[i];
}

void NoBias::setSensorData(const std::vector<double> & acc,
                                 const std::vector<double> & acc_inc,
                                 const std::vector<double> & ang_vel){

  // Orientation
  dynacore::Quaternion delt_quat;
  dynacore::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = (ang_vel[i]) * mercury::servo_rate;
    theta += delta_th[i] * delta_th[i];
  }

  delt_quat.w() = cos(theta/2.);
  delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
  delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
  delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;

  ori_pred_ = dynacore::QuatMultiply(global_ori_, delt_quat);

  for(int i(0); i<3; ++i){
    x_pred_[i] = x_[i] + x_[i+3] * mercury::servo_rate; // Velocity
    x_pred_[i+3] = x_[i+3]; // Acceleration
  }

  // Propagate Covariance
  Eigen::Matrix3d RotMtx(global_ori_);
  F_.setIdentity();
  F_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * mercury::servo_rate;
  // dynacore::pretty_print((dynacore::Matrix)F_, std::cout, "F");
  P_pred_ = F_ * P_ * F_.transpose() + Q_;

  // Update Observation
  dynacore::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  dynacore::Vect3 local_acc = x_pred_.segment(3,3) + grav;
  local_acc = RotMtx.transpose() * local_acc;

  for(int i(0); i<3; ++i) {
    // Sensed
    s_[i] = acc[i];
    s_[i+3] = acc_inc[i];
    // Observation
    h_[i] = local_acc[i];
    h_[i+3] = local_acc[i];
  }
  y_ = s_ - h_;
  // dynacore::pretty_print((dynacore::Vector)s_, std::cout, "s");
  // dynacore::pretty_print((dynacore::Vector)h_, std::cout, "h");

  Eigen::Matrix3d a_g_skew; a_g_skew.setZero();
  dynacore::Vect3 state_acc = x_.segment(3,3);
  a_g_skew(0, 1) = -(state_acc[2] + 9.81);   a_g_skew(0, 2) = state_acc[1];
  a_g_skew(1, 0) = (state_acc[2] + 9.81);   a_g_skew(1, 2) = -state_acc[0];
  a_g_skew(2, 0) = -state_acc[1];   a_g_skew(2, 1) = state_acc[0];

  H_.setZero();
  H_.block<3,3>(0,3) = RotMtx.transpose();
  H_.block<3,3>(0,6) = RotMtx.transpose() * a_g_skew;
  H_.block<3,3>(3,3) = RotMtx.transpose();
  H_.block<3,3>(3,6) = RotMtx.transpose() * a_g_skew;

  // dynacore::pretty_print((dynacore::Matrix)a_g_skew, std::cout, "ag skew");
  // dynacore::pretty_print((dynacore::Matrix)H_, std::cout, "H");

  dynacore::Matrix S = R_ + H_ * P_pred_ * H_.transpose();
  // dynacore::Matrix K = P_pred_ * H_.transpose() * S.inverse();
  dynacore::Matrix Sinv;
  dynacore::pseudoInverse(S, 0.0001, Sinv);
  dynacore::Matrix K = P_pred_ * H_.transpose() * Sinv;

  dynacore::Vector delta = K*y_;
  x_pred_ += delta.head(6);

  dynacore::Vect3 delta_ori = delta.tail(3);
  dynacore::Quaternion quat_delta;
  dynacore::convert(delta_ori, quat_delta);
  ori_pred_ = dynacore::QuatMultiply(quat_delta, ori_pred_);

  // dynacore::pretty_print(x_, std::cout, "x");
  // dynacore::pretty_print(S, std::cout, "S");
  // dynacore::pretty_print(K, std::cout, "K");
  // dynacore::pretty_print(delta, std::cout, "delta");
  // dynacore::pretty_print(ori_pred_, std::cout, "ori updated");

  dynacore::Matrix eye(DIM_STATE_NO_BIAS, DIM_STATE_NO_BIAS);
  eye.setIdentity();
  P_pred_ = (eye - K * H_) * P_pred_;

  // Set Angular Velocity
  _SetGlobalAngularVelocity(ang_vel);
  global_ori_ = ori_pred_;
  P_ = P_pred_;
  x_ = x_pred_;
}


void NoBias::_SetGlobalAngularVelocity(const std::vector<double> & ang_vel){
  dynacore::Quaternion ang_quat;
  ang_quat.w() = 0.;
  ang_quat.x() = ang_vel[0];
  ang_quat.y() = ang_vel[1];
  ang_quat.z() = ang_vel[2];

  dynacore::Quaternion quat_dot = dynacore::QuatMultiply(global_ori_, ang_quat, false);
  quat_dot = dynacore::QuatMultiply(quat_dot, global_ori_.inverse(), false);

  global_ang_vel_[0] = quat_dot.x();
  global_ang_vel_[1] = quat_dot.y();
  global_ang_vel_[2] = quat_dot.z();

}

#include "LIPM_KalmanFilter.hpp"
#include <Configuration.h>
#include <ParamHandler/ParamHandler.hpp>
#include <string>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

LIPM_KalmanFilter::LIPM_KalmanFilter():CoMStateEstimator(),
                                       prediction_var_(LIPM_KFILTER_STATE_DIM),
                                       observation_var_(LIPM_KFILTER_OBS_DIM)
{
  x_.setZero();
  F_.setIdentity(); //Prediction (dynamics) Matrix
  H_.setIdentity(); // Observation Matrix

  Q_.setIdentity();
  R_.setIdentity();

  _ParameterSetting();

  // Tuning
  for(int i(0); i<LIPM_KFILTER_STATE_DIM; ++i) Q_(i,i) = prediction_var_[i];
  for(int i(0); i<LIPM_KFILTER_OBS_DIM; ++i) R_(i,i) = observation_var_[i];
  eye_.setIdentity();

  x_.setZero();
  x_pre_.setZero();

  P_.setIdentity();
  P_pre_.setIdentity();
}

LIPM_KalmanFilter::~LIPM_KalmanFilter(){
  
}

void LIPM_KalmanFilter::EstimatorInitialization(const dynacore::Vector & com_state){
  x_.setZero();

  // position set by the current
  for(int i(0); i<2; ++i) x_[i] = com_state[i];
  x_pre_.setZero();
  P_.setIdentity();
  P_pre_.setIdentity();
}

void LIPM_KalmanFilter::InputData(const dynacore::Vector & com_state){
  // x, y, xdot, ydot

  // Prediction
  x_pre_ = F_ * x_;
  for(int i(0); i<2; ++i){
    x_pre_[i] += x_pre_[i+2] * mercury::servo_rate;
    x_pre_[i+2] += g_/h_* (x_[i]) * mercury::servo_rate;
  }

  P_pre_ = F_ * P_ * F_.transpose() + Q_;

  // Update
  dynacore::Vector y = com_state - H_ * x_pre_;
  S_ = R_ + H_ * P_pre_ * H_.transpose();
  K_ = P_pre_ * H_.transpose() * S_.inverse();

  x_ = x_pre_ + K_ * y;
  P_ = (eye_ - K_ * H_) * P_pre_;
}

void LIPM_KalmanFilter::Output(dynacore::Vector & est_state){
  est_state = x_;
}

void LIPM_KalmanFilter::_ParameterSetting(){
  ParamHandler handler(MercuryConfigPath"ESTIMATOR_com_state.yaml");

  handler.getVector("prediction_variance", prediction_var_);
  handler.getVector("observation_variance", observation_var_);

}

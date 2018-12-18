#include "BodyFootKalmanFilter.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/utilities.hpp>

BodyFootKalmanFilter::BodyFootKalmanFilter():
    dt_(mercury::servo_rate)
{
    state_ = dynacore::Vector::Zero(num_state);
    state_pred_ = dynacore::Vector::Zero(num_state);
    obs_ = dynacore::Vector::Zero(num_obs);
    // Prediction
    F_ = dynacore::Matrix::Identity(num_state, num_state);
    P_ = dynacore::Matrix::Identity(num_state, num_state);
    Q_ = dynacore::Matrix::Identity(num_state, num_state);
    //F_.block(0,10, 2, 2) = dynacore::Matrix::Identity(2,2) * dt_;

    // dynacore::pretty_print(F_, std::cout, "F matx");
    // Observation
    R_ = dynacore::Matrix::Identity(num_obs, num_obs);
    H_ = dynacore::Matrix::Identity(num_obs, num_state);

    sp_ = Mercury_StateProvider::getStateProvider();
}

BodyFootKalmanFilter::~BodyFootKalmanFilter(){
}

void BodyFootKalmanFilter::Initialization(void* obs, double height){
    body_height_ = height;
    omega_ = 9.81/body_height_;
    BodyFootObs* _obs = ((BodyFootObs*)obs);

    if(_obs->led_visible_[0]) state_.head(2) = _obs->body_led_pos_;
    if(_obs->led_visible_[1]) state_.segment(2, 2) = _obs->rfoot_out_led_pos_;
    if(_obs->led_visible_[2]) state_.segment(4, 2) = _obs->rfoot_in_led_pos_;
    if(_obs->led_visible_[3]) state_.segment(6, 2) = _obs->lfoot_out_led_pos_;
    if(_obs->led_visible_[4]) state_.segment(8, 2) = _obs->lfoot_in_led_pos_;

}
void BodyFootKalmanFilter::Estimation(void* obs_input, void * pred_input){
    BodyFootObs* _obs_input = ((BodyFootObs*)obs_input);

    for(int i(0); i<2; ++i){
        // Body Pos
        obs_[i] = _obs_input->body_led_pos_[i];
        // Right foot
        obs_[i+2] = _obs_input->rfoot_out_led_pos_[i];
        obs_[i+4] = _obs_input->rfoot_in_led_pos_[i];

        // Left foot
        obs_[i+6] = _obs_input->lfoot_out_led_pos_[i];
        obs_[i+8] = _obs_input->lfoot_in_led_pos_[i];

        // Body vel
        obs_[i + 10] = _obs_input->body_led_vel_[i];
    }
    // LED visibility check
    for (int i(0); i<num_led; ++i){
        if(_obs_input->led_visible_[i]>0){
            R_.block(2*i, 2*i, 2, 2) = 1. * dynacore::Matrix::Identity(2,2);
        }else {
            R_.block(2*i, 2*i, 2, 2) = 100000. * dynacore::Matrix::Identity(2,2);
        }
    }
    _Predict(pred_input);
    _Update();
}

void BodyFootKalmanFilter::getBodyPos(dynacore::Vect3 & body_pos){
    for(int i(0); i<2; ++i) {
        body_pos[i] = state_[i] - stance_loc_[i];
    }
    // TEST
    body_pos[2] = body_height_;
}

void BodyFootKalmanFilter::getFootPos(dynacore::Vect3 & rfoot_pos, dynacore::Vect3 & lfoot_pos){
    for(int i(0); i<2; ++i){
        rfoot_pos[i] = 0.5 * (state_[i + 2] + state_[i + 4]);
        lfoot_pos[i] = 0.5 * (state_[i + 6] + state_[i + 8]);
    }
}

void BodyFootKalmanFilter::_Predict(void* input){
    BodyFootInput* _pred_input = ((BodyFootInput*)input);
    stance_loc_.setZero();

    if( _pred_input->stance_foot_idx_ == mercury_link::rightFoot ){
        stance_foot_state_idx_ = 1;
    }else {  
        stance_foot_state_idx_ = 3;
    }

    for (int i(0); i < 2; ++i){
        stance_loc_ += 0.5 * state_.segment((stance_foot_state_idx_ + i)*2, 2);
    }
    // State Prediction
    state_pred_.head(2) = state_.head(2) + state_.segment(10, 2) * dt_;

    // Right Foot
    state_pred_.segment(2, 2) = state_.segment(2, 2) 
        + _pred_input->rfoot_out_led_vel_ * dt_;
    state_pred_.segment(4, 2) = state_.segment(4, 2) 
        + _pred_input->rfoot_in_led_vel_ * dt_;

    // Left Foot
    state_pred_.segment(6, 2) = state_.segment(6, 2) 
        + _pred_input->lfoot_out_led_vel_ * dt_;
    state_pred_.segment(8, 2) = state_.segment(8, 2) 
        + _pred_input->lfoot_in_led_vel_ * dt_;

    // TEST body led offset
    dynacore::Vect2 body_led_offset; body_led_offset.setZero();
    body_led_offset[0] = -0.077;
   state_pred_.tail(2) = state_.tail(2) + omega_ * 
       (state_.head(2) + body_led_offset - stance_loc_)* dt_;

    // Covariance update
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void BodyFootKalmanFilter::_Update(){
    dynacore::Matrix eye = dynacore::Matrix::Identity(num_state, num_state);

    // Observation
    dynacore::Vector y = obs_ - H_ * state_pred_;
    dynacore::Matrix S = R_ + H_ * P_ * H_.transpose();
    dynacore::Matrix K = P_* H_.transpose() * S.inverse();
    state_ = state_pred_ + K * y;
    P_ = (eye - K * H_) * P_ * ( eye - K * H_ ).transpose() + K * R_ * K.transpose();

}

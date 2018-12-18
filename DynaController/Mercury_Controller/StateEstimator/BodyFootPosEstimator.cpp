#include "BodyFootPosEstimator.hpp"
#include <Mercury_Controller/MoCapManager.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include "BodyFootKalmanFilter.hpp"

BodyFootPosEstimator::BodyFootPosEstimator(const RobotSystem* robot)
{
    mocap_manager_ = new MoCapManager(robot);
    mocap_manager_->start();

    robot_sys_ = robot;

    body_led_vel_.setZero();
    for(int i(0); i<3 ; ++i){
        vel_filter_.push_back(new deriv_lp_filter(2.*50.*M_PI, mercury::servo_rate));
    }
    body_foot_kalman_filter_ = new BodyFootKalmanFilter();
    kalman_obs_ = new BodyFootObs();
    kalman_input_ = new BodyFootInput();

    DataManager::GetDataManager()->RegisterData(&body_led_vel_, VECT3, "Body_LED_vel", 3);
    sp_ = Mercury_StateProvider::getStateProvider();
}

BodyFootPosEstimator::~BodyFootPosEstimator(){
    delete mocap_manager_;
    delete kalman_obs_;
    delete kalman_input_;
    delete body_foot_kalman_filter_;
}

void BodyFootPosEstimator::getMoCapBodyPos(const dynacore::Quaternion& body_ori, 
        dynacore::Vect3 & local_body_pos){
    
    body_foot_kalman_filter_->getBodyPos(local_body_pos);

    // Body LED offset accunt
    Eigen::Matrix3d Body_rot(body_ori);
    dynacore::Vect3 body_led_offset;
    body_led_offset.setZero();
    body_led_offset[0] = -0.075;
    local_body_pos += Body_rot * body_led_offset;
}

void BodyFootPosEstimator::Update(){
    for(int i(0); i<3; ++i){
        vel_filter_[i]->input(mocap_manager_->led_pos_data_[i]);
        body_led_vel_[i] = vel_filter_[i]->output();
    }
    
    _KalmanFilterOberservationSetup();
    _KalmanFilterPredictionInputSetup();
    body_foot_kalman_filter_->Estimation(kalman_obs_, kalman_input_);
}

void BodyFootPosEstimator::_KalmanFilterPredictionInputSetup(){
    dynacore::Vect3 lin_vel;
    
    // Right Foot
    robot_sys_->getLinearVel(mercury_link::LED_RLEG_3, lin_vel);
    kalman_input_->rfoot_out_led_vel_ = lin_vel.head(2);
    robot_sys_->getLinearVel(mercury_link::LED_RLEG_4, lin_vel);
    kalman_input_->rfoot_in_led_vel_ = lin_vel.head(2);

    // Left Foot
    robot_sys_->getLinearVel(mercury_link::LED_LLEG_3, lin_vel);
    kalman_input_->lfoot_out_led_vel_ = lin_vel.head(2);
    robot_sys_->getLinearVel(mercury_link::LED_LLEG_4, lin_vel);
    kalman_input_->lfoot_in_led_vel_ = lin_vel.head(2);

    kalman_input_->stance_foot_idx_ = sp_->stance_foot_;
}
void BodyFootPosEstimator::Initialization(const dynacore::Quaternion & body_ori){
    mocap_manager_->imu_body_ori_ = body_ori;
    mocap_manager_->CoordinateUpdateCall();

    _KalmanFilterOberservationSetup();
    // Height
    body_foot_kalman_filter_->Initialization(kalman_obs_, sp_->Q_[2]);
}

void BodyFootPosEstimator::_KalmanFilterOberservationSetup(){
    for(int i(0); i<2; ++i){
        kalman_obs_->body_led_pos_[i] = mocap_manager_->led_pos_data_[i];
        kalman_obs_->rfoot_out_led_pos_[i] = mocap_manager_->led_pos_data_[3*idx_rfoot_out + i];
        kalman_obs_->rfoot_in_led_pos_[i] = mocap_manager_->led_pos_data_[3*idx_rfoot_in + i];
        kalman_obs_->lfoot_out_led_pos_[i] = mocap_manager_->led_pos_data_[3*idx_lfoot_out + i];
        kalman_obs_->lfoot_in_led_pos_[i] = mocap_manager_->led_pos_data_[3*idx_lfoot_in + i];
        kalman_obs_->body_led_vel_[i] = body_led_vel_[i];
    }

    kalman_obs_->led_visible_ = dynacore::Vector::Zero(5);
    if(mocap_manager_->marker_cond_[0] > 0){ kalman_obs_->led_visible_[0] = 1.5; }
    else {kalman_obs_->led_visible_[0] = 0.0; }

    if(mocap_manager_->marker_cond_[idx_rfoot_out] > 0){ kalman_obs_->led_visible_[1] = 1.5; }
    else {kalman_obs_->led_visible_[1] = 0.0; }
    if(mocap_manager_->marker_cond_[idx_rfoot_in] > 0){ kalman_obs_->led_visible_[2] = 1.5; }
    else {kalman_obs_->led_visible_[2] = 0.0; }

    if(mocap_manager_->marker_cond_[idx_lfoot_out] > 0){ kalman_obs_->led_visible_[3] = 1.5; }
    else {kalman_obs_->led_visible_[3] = 0.0; }
    if(mocap_manager_->marker_cond_[idx_lfoot_in] > 0){ kalman_obs_->led_visible_[4] = 1.5; }
    else {kalman_obs_->led_visible_[4] = 0.0; }

}

void BodyFootPosEstimator::getMoCapBodyOri(dynacore::Quaternion & quat){
    quat = mocap_manager_->body_quat_;
}

void BodyFootPosEstimator::getMoCapBodyVel(dynacore::Vect3 & body_vel){	
    body_vel = body_led_vel_;
}

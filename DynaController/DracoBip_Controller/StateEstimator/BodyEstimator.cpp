#include "BodyEstimator.hpp"
#include <DracoBip_Controller/DracoBip_MoCapManager.hpp>
#include <Utils/DataManager.hpp>
#include <DracoBip/DracoBip_Definition.h>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>

BodyEstimator::BodyEstimator(const RobotSystem* robot)
{
    mocap_manager_ = new DracoBip_MoCapManager(robot);
    mocap_manager_->start();

    robot_sys_ = robot;

    body_led_vel_.setZero();
    for(int i(0); i<3 ; ++i){
        vel_filter_.push_back(new deriv_lp_filter(2.*50.*M_PI, dracobip::servo_rate));
    }

    DataManager::GetDataManager()->RegisterData(&body_led_vel_, VECT3, "Body_LED_vel", 3);
    sp_ = DracoBip_StateProvider::getStateProvider();
}

BodyEstimator::~BodyEstimator(){
    delete mocap_manager_;
}

void BodyEstimator::getMoCapBodyPos(const dynacore::Quaternion& body_ori, 
        dynacore::Vect3 & local_body_pos){
    

    // Body LED offset accunt
    Eigen::Matrix3d Body_rot(body_ori);
    dynacore::Vect3 body_led_offset;
    body_led_offset.setZero();
    body_led_offset[0] = -0.075;
    local_body_pos += Body_rot * body_led_offset;
}

void BodyEstimator::Update(){
    for(int i(0); i<3; ++i){
        vel_filter_[i]->input(mocap_manager_->led_pos_data_[i]);
        body_led_vel_[i] = vel_filter_[i]->output();
    }
}


void BodyEstimator::Initialization(const dynacore::Quaternion & body_ori){
    mocap_manager_->imu_body_ori_ = body_ori;
    mocap_manager_->CoordinateUpdateCall();

}

void BodyEstimator::getMoCapBodyOri(dynacore::Quaternion & quat){
    quat = mocap_manager_->body_quat_;
}

void BodyEstimator::getMoCapBodyVel(dynacore::Vect3 & body_vel){	
    body_vel = body_led_vel_;
}

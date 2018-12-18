#include "DracoBip_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "DracoBip_StateProvider.hpp"
#include "DracoBip_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <DracoBip/DracoBip_Model.hpp>

// Test SET LIST
#include <DracoBip_Controller/TestSet/JointCtrlTest.hpp>
#include <DracoBip_Controller/TestSet/BodyCtrlTest.hpp>
#include <DracoBip_Controller/TestSet/WalkingConfigTest.hpp>

DracoBip_interface::DracoBip_interface():
    interface(),
    torque_(dracobip::num_act_joint),
    torque_command_(dracobip::num_act_joint),
    jpos_command_(dracobip::num_act_joint),
    jvel_command_(dracobip::num_act_joint),
    temperature_(dracobip::num_act_joint),
    motor_current_(dracobip::num_act_joint),
    waiting_count_(10)
{

    robot_sys_ = new DracoBip_Model();
    jpos_command_.setZero();
    jvel_command_.setZero();
    torque_command_.setZero();

    test_cmd_ = new DracoBip_Command();
    sp_ = DracoBip_StateProvider::getStateProvider();
    state_estimator_ = new DracoBip_StateEstimator(robot_sys_);  
    
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
     DataManager::GetDataManager()->RegisterData(
            &torque_, DYN_VEC, "torque", dracobip::num_act_joint);
    
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", dracobip::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", dracobip::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", dracobip::num_act_joint);

    // For save
    temperature_.setZero();
    motor_current_.setZero();
    DataManager::GetDataManager()->RegisterData(
            &temperature_, DYN_VEC, "temperature", dracobip::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &motor_current_, DYN_VEC, "motor_current", dracobip::num_act_joint);

    _ParameterSetting();
    
    printf("[DracoBip_interface] Contruct\n");
}

DracoBip_interface::~DracoBip_interface(){
    delete test_;
    delete state_estimator_;
}

void DracoBip_interface::GetCommand( void* _data, void* _command){
    DracoBip_Command* cmd = ((DracoBip_Command*)_command);
    DracoBip_SensorData* data = ((DracoBip_SensorData*)_data);

    if(!_Initialization(data, cmd)){
        state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
        
        stop_test_ = _UpdateTestCommand(test_cmd_);
        if(stop_test_){
            _SetStopCommand(data, cmd);
        }else {
            _CopyCommand(cmd);
        }
     }    

    // Save Data
    for(int i(0); i<dracobip::num_act_joint; ++i){
        torque_[i] = data->torque[i];
        temperature_[i] = data->temperature[i];
        motor_current_[i] = data->motor_current[i];
    }

    running_time_ = (double)(count_) * dracobip::servo_rate;
    ++count_;
    sp_->curr_time_ = running_time_;
}

bool DracoBip_interface::_UpdateTestCommand(DracoBip_Command* test_cmd){

    bool over_limit(false);
    for(int i(0); i<dracobip::num_act_joint; ++i){
        // JPos limit check
        if(test_cmd->jpos_cmd[i] > jpos_max_[i]) jpos_command_[i] = jpos_max_[i];
        else if(test_cmd->jpos_cmd[i] < jpos_min_[i]) jpos_command_[i] = jpos_min_[i];
        else jpos_command_[i] = test_cmd->jpos_cmd[i];

        // Velocity limit
        if(test_cmd->jvel_cmd[i] > jvel_max_[i]) over_limit = true;
        else if(test_cmd->jvel_cmd[i] < jvel_min_[i]) over_limit = true;
        else jvel_command_[i] = test_cmd->jvel_cmd[i];

        // Torque limit
        if(test_cmd->jtorque_cmd[i] > trq_max_[i]) over_limit = true;
        else if(test_cmd->jtorque_cmd[i] < trq_min_[i]) over_limit = true;
        else torque_command_[i] = test_cmd->jtorque_cmd[i];
    }
    return over_limit;
}

void DracoBip_interface::_SetStopCommand( DracoBip_SensorData* data, 
                                          DracoBip_Command* cmd){
    for(int i(0); i<dracobip::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = 0.;
        cmd->jpos_cmd[i] = data->jpos[i];
        cmd->jvel_cmd[i] = 0.;
    }
}

void DracoBip_interface::_CopyCommand(DracoBip_Command* cmd ){
    for(int i(0); i< dracobip::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = torque_command_[i];
        cmd->jpos_cmd[i] = jpos_command_[i];
        cmd->jvel_cmd[i] = jvel_command_[i];
    }
}

bool DracoBip_interface::_Initialization(DracoBip_SensorData* data, DracoBip_Command* cmd){
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[DracoBip Interface] Test initialization is done\n");
    }
    if(count_ < waiting_count_){
        _SetStopCommand(data, cmd);
        state_estimator_->Initialization(data);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void DracoBip_interface::_ParameterSetting(){
    ParamHandler handler(DracoBipConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Basic Test ***********************************
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
    // Walking Test ***********************************
    }else if(tmp_string == "walking_test"){
        test_ = new WalkingConfigTest(robot_sys_);
    // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyCtrlTest(robot_sys_);
        // Stance and Swing Test ***********************************
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }

    handler.getVector("jpos_max", jpos_max_);
    handler.getVector("jpos_min", jpos_min_);

    handler.getVector("jvel_max", jvel_max_);
    handler.getVector("jvel_min", jvel_min_);
    
    handler.getVector("trq_max", trq_max_);
    handler.getVector("trq_min", trq_min_);
    
    printf("[DracoBip_interface] Parameter setup is done\n");
}

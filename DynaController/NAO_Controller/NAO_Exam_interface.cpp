#include "NAO_Exam_interface.hpp"
#include <NAO_Exam_Controller/TestSet/JointCtrlTest.hpp>
#include <NAO/NAO_Model.hpp>
#include "NAO_Exam_StateProvider.hpp"

NAO_Exam_interface::NAO_Exam_interface():interface(), gamma_(nao::num_act_joint){
    robot_sys_ = new NAO_Model();
    test_ = new JointCtrlTest(robot_sys_);
    gamma_.setZero();
    sp_ = NAO_Exam_StateProvider::getStateProvider();

    printf("[NAO_Exam_interface] Contruct\n");
}

NAO_Exam_interface::~NAO_Exam_interface(){
}

void NAO_Exam_interface::GetCommand(void* sensor_data, std::vector<double> & command){
    NAO_Exam_SensorData* data = ((NAO_Exam_SensorData*) sensor_data);
    for(int i(0); i<nao::num_qdot; ++i) {
        sp_->q_[i] = data->q[i];
        sp_->qdot_[i] = data->qdot[i];
    }
    sp_->q_[nao::num_qdot] = 1.;
    robot_sys_->UpdateSystem(sp_->q_, sp_->qdot_);

   if(!_Initialization(sensor_data)){
        test_->getTorqueInput(gamma_);
    }

   for (int i(0); i<nao::num_act_joint; ++i) command[i] = gamma_[i];

    ++count_;
    running_time_ = count_ * nao::servo_rate;
}

bool NAO_Exam_interface::_Initialization(void * sensor_data){

    if(count_ < 1){
        gamma_.setZero();
        return true;
    }
    return false;
}


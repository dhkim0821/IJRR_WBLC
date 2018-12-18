#include "DracoBip_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "DracoBip_DynaCtrl_Definition.h"
#include <DracoBip/DracoBip_Model.hpp>

DracoBip_StateProvider* DracoBip_StateProvider::getStateProvider(){
    static DracoBip_StateProvider state_provider_;
    return &state_provider_;
}

DracoBip_StateProvider::DracoBip_StateProvider():
                                stance_foot_(dracobip_link::lAnkle),
                                Q_(dracobip::num_q),
                                Qdot_(dracobip::num_qdot),
                                rotor_inertia_(dracobip::num_act_joint),
                                b_rfoot_contact_(0),
                                b_lfoot_contact_(0),
                                reaction_forces_(10)
{
    rotor_inertia_.setZero();
    Q_.setZero();
    Qdot_.setZero();
    global_pos_local_.setZero();
    reaction_forces_.setZero();
    des_location_.setZero();

    rfoot_pos_.setZero();
    lfoot_pos_.setZero();
    rfoot_vel_.setZero();
    lfoot_vel_.setZero();

    est_mocap_body_vel_.setZero();
    DataManager* data_manager = DataManager::GetDataManager();

    data_manager->RegisterData(&curr_time_, DOUBLE, "time");
    data_manager->RegisterData(&Q_, DYN_VEC, "config", dracobip::num_q);
    data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", dracobip::num_qdot);
    data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

    data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);

    data_manager->RegisterData(&reaction_forces_, DYN_VEC, "reaction_force", 10);

   data_manager->RegisterData(&rfoot_pos_, VECT3, "rfoot_pos", 3); 
   data_manager->RegisterData(&lfoot_pos_, VECT3, "lfoot_pos", 3); 
   data_manager->RegisterData(&rfoot_vel_, VECT3, "rfoot_vel", 3); 
   data_manager->RegisterData(&lfoot_vel_, VECT3, "lfoot_vel", 3); 

   data_manager->RegisterData(&est_mocap_body_vel_, VECT2, "est_mocap_body_vel",2);
}

void DracoBip_StateProvider::SaveCurrentData(const RobotSystem* robot_sys){
    robot_sys->getPos(dracobip_link::rAnkle, rfoot_pos_);
    robot_sys->getPos(dracobip_link::lAnkle, lfoot_pos_);
    robot_sys->getPos(dracobip_link::rAnkle, rfoot_vel_);
    robot_sys->getPos(dracobip_link::lAnkle, lfoot_vel_);
}

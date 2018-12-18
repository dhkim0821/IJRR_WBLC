#include "Valkyrie_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "Valkyrie_DynaCtrl_Definition.h"

Valkyrie_StateProvider* Valkyrie_StateProvider::getStateProvider(){
    static Valkyrie_StateProvider state_provider_;
    return &state_provider_;
}

Valkyrie_StateProvider::Valkyrie_StateProvider():
                                stance_foot_(valkyrie_link::leftFoot),
                                Q_(valkyrie::num_q),
                                Qdot_(valkyrie::num_qdot),
                                b_rfoot_contact_(0),
                                b_lfoot_contact_(0)
{
  Q_.setZero();
  Qdot_.setZero();
  global_pos_local_.setZero();
  global_cup_pos_.setZero();

  des_location_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, DYN_VEC, "config", valkyrie::num_q);
  data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", valkyrie::num_qdot);
  data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

  data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
  data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);
}


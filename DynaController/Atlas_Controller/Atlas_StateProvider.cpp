#include "Atlas_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "Atlas_DynaCtrl_Definition.h"

Atlas_StateProvider* Atlas_StateProvider::getStateProvider(){
    static Atlas_StateProvider state_provider_;
    return &state_provider_;
}

Atlas_StateProvider::Atlas_StateProvider():
                                stance_foot_(atlas_link::leftFoot),
                                Q_(atlas::num_q),
                                Qdot_(atlas::num_qdot),
                                b_rfoot_contact_(0),
                                b_lfoot_contact_(0)
{
  Q_.setZero();
  Qdot_.setZero();
  global_pos_local_.setZero();

  des_location_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, DYN_VEC, "config", atlas::num_q);
  data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", atlas::num_qdot);
  data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

  data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
  data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);
}


#ifndef STATE_PROVIDER_NAO
#define STATE_PROVIDER_NAO

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace dynacore;

class RobotSystem;

class NAO_StateProvider{
public:
  static NAO_StateProvider* getStateProvider();
  ~NAO_StateProvider(){}

  dynacore::Vect3 imu_ang_vel_;
  // Important!!!!!!!!
  int stance_foot_;

  double curr_time_;

  Vector Q_;
  Vector Qdot_;
  Vector jpos_ini_;
  Vector des_jpos_prev_;
   
  dynacore::Vect3 global_pos_local_;
  dynacore::Vect2 des_location_;

  int b_rfoot_contact_;
  int b_lfoot_contact_;
  int num_step_copy_;

private:
  NAO_StateProvider();
};


#endif

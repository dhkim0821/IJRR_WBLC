#ifndef STATE_PROVIDER_MERCURY
#define STATE_PROVIDER_MERCURY

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "Mercury_DynaControl_Definition.h"

using namespace dynacore;
class RobotSystem;

class Mercury_StateProvider{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Mercury_StateProvider* getStateProvider();
  ~Mercury_StateProvider(){}

  double des_body_pitch_; // (rad)
  // Walking related data
  dynacore::Vector des_jpos_prev_;
  dynacore::Vector mjpos_;
  
  RobotSystem* jjpos_robot_sys_;
  dynacore::Vector jjpos_config_;
  dynacore::Vector jjvel_qdot_;
  dynacore::Vect3 jjpos_rfoot_pos_;
  dynacore::Vect3 jjpos_lfoot_pos_;

  dynacore::Vect3 body_ori_rpy_;
  dynacore::Vect3 imu_acc_inc_;
  dynacore::Vect3 imu_ang_vel_;
  dynacore::Vect3 imu_acc_;
  // Important!!!!!!!!
  int stance_foot_;
  Vector Q_;
  Vector Qdot_;

  double first_LED_x_;
  double first_LED_y_;

  bool initialized_;
  double curr_time_;

  int b_rfoot_contact_;
  int b_lfoot_contact_;
  int phase_copy_;
  int num_step_copy_;

  void SaveCurrentData(Mercury_SensorData* data,  RobotSystem* ctrl_robot);

  dynacore::Vector reaction_forces_;
  dynacore::Vector qddot_cmd_;

  dynacore::Vect3 global_pos_local_;
  dynacore::Vect3 global_jjpos_local_;
  
  dynacore::Vect2 des_location_;

  dynacore::Vect3 CoM_pos_;
  dynacore::Vect3 CoM_vel_;
  dynacore::Vect2 est_CoM_vel_;
  
  dynacore::Vect2 est_mocap_body_vel_;
  dynacore::Vect3 est_mocap_body_pos_;

  dynacore::Vect3 com_pos_des_;
  dynacore::Vect3 com_vel_des_;

  dynacore::Vect2 average_vel_;
  dynacore::Vector rotor_inertia_;

  // (x, y, x_dot, y_dot)
  dynacore::Vector estimated_com_state_;
  dynacore::Vector com_state_imu_; // (x, y, xdot, ydot, xddot, yddot)
  
  dynacore::Vect3 Rfoot_pos_;
  dynacore::Vect3 Rfoot_vel_;
  dynacore::Vect3 Lfoot_pos_;
  dynacore::Vect3 Lfoot_vel_;

  dynacore::Vect3 sim_imu_pos;
  dynacore::Vect3 sim_imu_vel;

  dynacore::Vector led_kin_data_;

private:
  Mercury_StateProvider();
};


#endif

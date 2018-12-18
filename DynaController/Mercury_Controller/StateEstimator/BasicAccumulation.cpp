#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>


BasicAccumulation::BasicAccumulation():OriEstimator(), com_state_(6){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  com_state_.setZero();

  // Bias Filter 
  bias_lp_frequency_cutoff = 2.0*3.1415*1.0; // 1Hz // (2*pi*frequency) rads/s 
  x_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  y_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  z_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);  
  x_acc_bias = 0.0;
  y_acc_bias = 0.0;  
  z_acc_bias = 0.0;

  imu_acc.setZero(); // initialize IMU acceleration data
  
  gravity_mag = 9.81; // m/s^2;
  theta_x = 0.0;
  theta_y = 0.0;
  // Initialize body orientation to identity.
  g_B_local_vec.setZero();
  dynacore::Vect3 rpy_init; rpy_init.setZero();
  dynacore::convert(rpy_init, Oq_B); 
  OR_B_init = Oq_B.toRotationMatrix();

}


BasicAccumulation::~BasicAccumulation(){}

void BasicAccumulation::CoMStateInitialization(
        const dynacore::Vect3 & com_pos, 
        const dynacore::Vect3 & com_vel){

    // com_state_[0] = com_pos[0];
    // com_state_[1] = com_pos[1];
    // com_state_[2] = com_vel[0];
    // com_state_[3] = com_vel[1];
}

void BasicAccumulation::getEstimatedCoMState(dynacore::Vector & com_state){
    com_state = com_state_;
}

void BasicAccumulation::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                                const std::vector<double> & acc,
                                                const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i){
      global_ang_vel_[i] = ang_vel[i];
    ini_acc_[i] = acc[i];
    }

  // Update bias estimate
  x_bias_low_pass_filter->input(acc[0]);
  y_bias_low_pass_filter->input(acc[1]);
  z_bias_low_pass_filter->input(acc[2]);

  x_acc_bias = x_bias_low_pass_filter->output();
  y_acc_bias = y_bias_low_pass_filter->output(); 
  z_acc_bias = z_bias_low_pass_filter->output();     

  //printf("Basic Accumulation \n");
  InitIMUOrientationEstimateFromGravity();
  //dynacore::pretty_print(Oq_B_init, std::cout, "Oq_B_init");
  global_ori_ = Oq_B_init;

}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){

  // Convert body omega into a delta quaternion ------------------------------
  dynacore::Vect3 body_omega; body_omega.setZero();
  for(size_t i = 0; i < 3; i++){
    body_omega[i] = ang_vel[i];
    //global_ang_vel_[i] = ang_vel[i];
  }
  dynacore::Quaternion delta_quat_body;
  dynacore::convert(body_omega*mercury::servo_rate, delta_quat_body);

  dynacore::Matrix R_global_to_imu = global_ori_.normalized().toRotationMatrix();
  global_ang_vel_ = R_global_to_imu * body_omega;
  // Perform orientation update via integration
  Oq_B = dynacore::QuatMultiply(Oq_B, delta_quat_body); 

  global_ori_ = Oq_B; // Use the stored  value

}

void BasicAccumulation::InitIMUOrientationEstimateFromGravity(){
  // Finds The orientation of the body with respect to the fixed frame O ((^OR_B ). 
  // This assumes that the IMU can only sense gravity as the acceleration.
  //
  // The algorithm attempts to solve ^OR_B * f_b = f_o, 
  // where f_b is the acceleration of gravity in the body frame.
  // f_o is the acceleration of gravity in the fixed frame
  // In addition to ^OR_B acting as a change of frame formula, 
  // note that ^OR_B will also equal to the orientation of the body w.r.t to the fixed frame.

  // We will rotate f_b using an extrinsic rotation with global R_x (roll) and R_y (pitch) rotations
  // Thus, we will perform ^OR_y ^OR_x f_b = f_o.

  // Finally, note that ^OR_b = ^OR_y ^OR_x.
  // The resulting orientation will have the xhat component of ^OR_b (ie: ^OR_b.col(0)) to be always planar 
  // with the inertial x-z plane. 
  //
  // It is best to visualize the extrinsic rotation on paper for any given extrinsic roll then pitch operations

  g_B.setZero();
  g_B[0] = -x_acc_bias; 
  g_B[1] = -y_acc_bias;
  g_B[2] = -z_acc_bias; // We expect a negative number if gravity is pointing opposite of the IMU zhat direction

  // Test Vector
  // f_b = [-0.057744 -0.001452  -0.998330]  
  // g_B[0] = -0.057744;
  // g_B[1] = -0.001452;
  // g_B[2] = -0.998330;  // We expect a negative number if gravity is pointing opposite of the IMU zhat direction
  // g_B *= 9.7;

  gravity_mag = g_B.norm();
  g_B /= gravity_mag;

  // dynacore::Quaternion q_world_Ry;
  // dynacore::Quaternion q_world_Rx;      

  // Prepare to rotate gravity vector
  g_B_local.w() = 0;
  g_B_local.x() = g_B[0];  g_B_local.y() = g_B[1]; g_B_local.z() = g_B[2];
  g_B_local_vec[0] = g_B[0];   g_B_local_vec[1] = g_B[1];   g_B_local_vec[2] = g_B[2];

  //---------------------------------------------------------------------------
  // Use Rx to rotate the roll and align gravity vector  -
  // Compute Roll to rotate
  // theta_x = atan(g_B[1]/g_B[2]);
  // double roll_val = theta_x;      

  // The following method can handle any initial vector due to gravity
  theta_x = atan2(g_B_local_vec[2], g_B_local_vec[1]); // Returns angle \in [-pi, pi] between z and y projected vectors.
  double roll_val = (-M_PI/2.0 - theta_x);      // (-pi/2 - theta_x)
  roll_value_comp = roll_val;

  //dynacore::convert(0.0, 0.0, roll_val, q_world_Rx);
  // Create Roll Quaternion
  q_world_Rx.w() = cos(roll_val/2.);;
  q_world_Rx.x() = sin(roll_val/2.);
  q_world_Rx.y() = 0;
  q_world_Rx.z() = 0;

  //Rotate gravity vector to align the yhat directions
  dynacore::Matrix Rx = q_world_Rx.normalized().toRotationMatrix();
  g_B_local_vec = Rx*g_B_local_vec;
  // Note that quat multiply sometimes wraps around...
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(
              q_world_Rx, g_B_local,false), q_world_Rx.inverse(), false);


  // Use Ry to rotate pitch and align gravity vector  ---------------------------
  // Compute Pitch to rotate
  // theta_y = atan(g_B_local.x()/g_B_local.z());
  // double pitch_val = -theta_y;

  // The following method can handle any initial vector due to gravity
  theta_y = atan2(g_B_local_vec[2], g_B_local_vec[0]); // Returns angle \in [-pi, pi] between z and x projected vectors.
  double pitch_val = -((-M_PI/2.0) - theta_y);   // This is actually -(-pi/2 - theta_y)
  pitch_value_comp = pitch_val;

  //dynacore::convert(0.0, pitch_val, 0.0, q_world_Ry);
  // Create Pitch Quaternion
  q_world_Ry.w() = cos(pitch_val/2.);
  q_world_Ry.x() = 0;
  q_world_Ry.y() = sin(pitch_val/2.);
  q_world_Ry.z() = 0;  

  // Rotate gravity vector to align the xhat directions
  dynacore::Matrix Ry = q_world_Ry.normalized().toRotationMatrix();
  g_B_local_vec = Ry*g_B_local_vec;  
  // Note that quat multiply sometimes wraps around...   
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(
              q_world_Ry, g_B_local, false), q_world_Ry.inverse(), false);

  // Obtain initial body orientation w.r.t fixed frame.
  //Oq_B = q_y * q_x * q_b
  Oq_B_init = dynacore::QuatMultiply(q_world_Ry, q_world_Rx);
  // Set rotation matrix
  OR_B_init = Oq_B_init.normalized().toRotationMatrix();

  // // // Initialize global orientation
  Oq_B = Oq_B_init;

}

#ifndef MERCURY_INTERFACE_H
#define MERCURY_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <Filter/filters.hpp>

class Mercury_StateEstimator;
class Mercury_StateProvider;
class ExtCtrlReceiver;

class Mercury_interface: public interface{
public:
  Mercury_interface();
  virtual ~Mercury_interface();

public:
  virtual void GetCommand(void * data, void * command);
  void GetReactionForce(std::vector<dynacore::Vect3> & reaction_force );

  dynacore::Quaternion global_ori_;

private:
  ExtCtrlReceiver* ext_ctrl_receiver_;
  int waiting_count_;
  double ramp_time_;
  
  std::vector<double> torque_limit_min_;
  std::vector<double> torque_limit_max_;
  std::vector<double> jpos_limit_min_;
  std::vector<double> jpos_limit_max_;
  std::vector<double> spring_const_;
  // std::vector<filter*> filter_jtorque_cmd_;
  
  void _ParameterSetting();
  bool _Initialization(Mercury_SensorData* );

  Mercury_Command* test_cmd_;
  // dynacore::Vector filtered_torque_command_;
  dynacore::Vector torque_command_;
  dynacore::Vector jpos_command_;
  dynacore::Vector jvel_command_;
  
  dynacore::Vector sensed_torque_;
  
  dynacore::Vector motor_current_;
  dynacore::Vector bus_current_;
  dynacore::Vector bus_voltage_;
  dynacore::Vector last_config_;

  bool b_last_config_update_;

  dynacore::Vector jjvel_;
  dynacore::Vector jjpos_;
  
  dynacore::Vector initial_jpos_;
  Mercury_StateEstimator* state_estimator_;
  Mercury_StateProvider* sp_;

  double walking_dist_;
  double walking_duration_;
  double walking_st_time_;

// step left
    double left1_walking_dist_;
  double left1_walking_duration_;
  

// step right
    double right_walking_dist_;
  double right_walking_duration_;
  

  // step left again
    double left2_walking_dist_;
  double left2_walking_duration_;

  // step back
    double backward_walking_dist_;
  double backward_walking_duration_;
};

#endif

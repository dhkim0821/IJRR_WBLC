#ifndef DRACO_BIPED_INTERFACE_H
#define DRACO_BIPED_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "DracoBip_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class DracoBip_StateEstimator;
class DracoBip_StateProvider;

class DracoBip_interface: public interface{
public:
  DracoBip_interface();
  virtual ~DracoBip_interface();
  virtual void GetCommand(void * data, void * command);

private:
  int waiting_count_;
  
  void _ParameterSetting();
  bool _Initialization(DracoBip_SensorData*, DracoBip_Command* );

  DracoBip_Command* test_cmd_;

  dynacore::Vector torque_;
 
  dynacore::Vector torque_command_;
  dynacore::Vector jpos_command_;
  dynacore::Vector jvel_command_;
  
  DracoBip_StateEstimator* state_estimator_;
  DracoBip_StateProvider* sp_;

  bool stop_test_;

  std::vector<double> jpos_max_;
  std::vector<double> jpos_min_;
 
  std::vector<double> jvel_max_;
  std::vector<double> jvel_min_;

  std::vector<double> trq_max_;
  std::vector<double> trq_min_;

  bool _UpdateTestCommand(DracoBip_Command* test_cmd);
  void _SetStopCommand( DracoBip_SensorData* data, DracoBip_Command* cmd);
  void _CopyCommand(DracoBip_Command* cmd );

  dynacore::Vector temperature_;
  dynacore::Vector motor_current_;


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

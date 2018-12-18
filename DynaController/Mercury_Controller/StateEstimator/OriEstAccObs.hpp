#ifndef ACCELERATION_OBSERVING_ESTIMATOR
#define ACCELERATION_OBSERVING_ESTIMATOR

#include "OriEstimator.hpp"
#define DIM_STATE_EST_ACC_OBS 3*4
#define DIM_OBSER_EST_ACC_OBS 3*2

class OriEstAccObs:public OriEstimator{
public:
  OriEstAccObs();
  virtual ~OriEstAccObs();

  virtual void EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                       const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel);

protected:
  Eigen::Matrix<double, DIM_STATE_EST_ACC_OBS, DIM_STATE_EST_ACC_OBS> F_;
  Eigen::Matrix<double, DIM_OBSER_EST_ACC_OBS, DIM_STATE_EST_ACC_OBS> H_;

  Eigen::Matrix<double, DIM_STATE_EST_ACC_OBS, DIM_STATE_EST_ACC_OBS> P_;
  Eigen::Matrix<double, DIM_STATE_EST_ACC_OBS, DIM_STATE_EST_ACC_OBS> P_pred_;

  Eigen::Matrix<double, DIM_STATE_EST_ACC_OBS, DIM_STATE_EST_ACC_OBS> Q_;
  Eigen::Matrix<double, DIM_OBSER_EST_ACC_OBS, DIM_OBSER_EST_ACC_OBS> R_;
  // vel(3), acc(3), bias_omega(3), ...[orientation (3*)]
  dynacore::Vector x_;
  dynacore::Vector x_pred_;
  //Observation
  Eigen::Matrix<double, DIM_OBSER_EST_ACC_OBS, 1> y_;
  Eigen::Matrix<double, DIM_OBSER_EST_ACC_OBS, 1> s_;
  Eigen::Matrix<double, DIM_OBSER_EST_ACC_OBS, 1> h_;

  // ori prime
  dynacore::Quaternion ori_pred_;


  void _SetGlobalAngularVelocity(const std::vector<double> & ang_vel);

};


#endif

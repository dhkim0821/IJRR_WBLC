#ifndef ACCELERATION_OBSERVING_NO_BIAS_ESTIMATOR
#define ACCELERATION_OBSERVING_NO_BIAS_ESTIMATOR

#include "OriEstimator.hpp"
#define DIM_STATE_NO_BIAS 3*3
#define DIM_OBSER_NO_BIAS 3*2

class NoBias:public OriEstimator{
public:
  NoBias();
  virtual ~NoBias();

  virtual void EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                       const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel);

protected:
  Eigen::Matrix<double, DIM_STATE_NO_BIAS, DIM_STATE_NO_BIAS> F_;
  Eigen::Matrix<double, DIM_OBSER_NO_BIAS, DIM_STATE_NO_BIAS> H_;

  Eigen::Matrix<double, DIM_STATE_NO_BIAS, DIM_STATE_NO_BIAS> P_;
  Eigen::Matrix<double, DIM_STATE_NO_BIAS, DIM_STATE_NO_BIAS> P_pred_;

  Eigen::Matrix<double, DIM_STATE_NO_BIAS, DIM_STATE_NO_BIAS> Q_;
  Eigen::Matrix<double, DIM_OBSER_NO_BIAS, DIM_OBSER_NO_BIAS> R_;
  // vel(3), acc(3),  ...[orientation (3*)]
  dynacore::Vector x_;
  dynacore::Vector x_pred_;
  //Observation
  Eigen::Matrix<double, DIM_OBSER_NO_BIAS, 1> y_;
  Eigen::Matrix<double, DIM_OBSER_NO_BIAS, 1> s_;
  Eigen::Matrix<double, DIM_OBSER_NO_BIAS, 1> h_;

  // ori prime
  dynacore::Quaternion ori_pred_;


  void _SetGlobalAngularVelocity(const std::vector<double> & ang_vel);

};


#endif

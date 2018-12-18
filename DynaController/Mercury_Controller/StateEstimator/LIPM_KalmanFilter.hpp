#ifndef LIPM_KALMAN_FILTER
#define LIPM_KALMAN_FILTER

#define LIPM_KFILTER_STATE_DIM 4
#define LIPM_KFILTER_OBS_DIM 4


#include "CoMStateEstimator.hpp"

class LIPM_KalmanFilter: public CoMStateEstimator{
public:
  LIPM_KalmanFilter();
  virtual ~LIPM_KalmanFilter();

  // x, y, xdot, ydot (assume xp, yp = 0)
  virtual void InputData(const dynacore::Vector& input);
  virtual void Output(dynacore::Vector & output);
  virtual void EstimatorInitialization(const dynacore::Vector & );

protected:
  void _ParameterSetting();
  std::vector<double> prediction_var_;
  std::vector<double> observation_var_;

  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, 1> x_;
  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, 1> x_pre_;

  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_STATE_DIM> F_;
  Eigen::Matrix<double, LIPM_KFILTER_OBS_DIM, LIPM_KFILTER_STATE_DIM> H_;
  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_STATE_DIM> P_;
  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_STATE_DIM> P_pre_;

  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_OBS_DIM> K_;
  Eigen::Matrix<double, LIPM_KFILTER_OBS_DIM, LIPM_KFILTER_OBS_DIM> S_;
  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_STATE_DIM> Q_;
  Eigen::Matrix<double, LIPM_KFILTER_OBS_DIM, LIPM_KFILTER_OBS_DIM> R_;

  Eigen::Matrix<double, LIPM_KFILTER_STATE_DIM, LIPM_KFILTER_STATE_DIM> eye_;
};
#endif

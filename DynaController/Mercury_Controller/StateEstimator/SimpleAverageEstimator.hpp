#ifndef SIMPLE_AVERAGE_ESTIMATOR
#define SIMPLE_AVERAGE_ESTIMATOR

#include <Filter/filters.hpp>

class SimpleAverageEstimator{
public:
  SimpleAverageEstimator();
  ~SimpleAverageEstimator();

  void Initialization(double xdot, double ydot);
  void Update(double xdot, double ydot); 
  void Output(double& xdot, double & ydot);

protected:
  void _SettingParameter();
  double xdot_est_;
  double ydot_est_;

  double t_const_;
  double xdot_limit_;
  double ydot_limit_;
  double dt_;
};

#endif

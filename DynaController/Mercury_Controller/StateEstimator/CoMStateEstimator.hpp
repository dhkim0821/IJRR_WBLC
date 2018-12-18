#ifndef COM_STATE_ESTIMATOR
#define COM_STATE_ESTIMATOR

#include <Utils/wrap_eigen.hpp>

class CoMStateEstimator{
public:
  CoMStateEstimator():g_(9.81){}
  virtual ~CoMStateEstimator(){}

  virtual void InputData(const dynacore::Vector& input) = 0;
  virtual void Output(dynacore::Vector & output) = 0;
  virtual void EstimatorInitialization(const dynacore::Vector & )= 0;

  double h_;
protected:
  double g_;
};
#endif

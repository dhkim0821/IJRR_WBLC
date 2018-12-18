#include "SimpleAverageEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <Configuration.h>
#include <math.h>

SimpleAverageEstimator::SimpleAverageEstimator():
    xdot_est_(0.),
    ydot_est_(0.),
    t_const_(0.12),
    xdot_limit_(3.0),
    ydot_limit_(1.0),
    dt_(mercury::servo_rate)
{
    _SettingParameter();
}

SimpleAverageEstimator::~SimpleAverageEstimator(){}

void SimpleAverageEstimator::Initialization(double xdot, double ydot){
    xdot_est_ = xdot;
    ydot_est_ = ydot;
}

void SimpleAverageEstimator::Update(double xdot, double ydot){
    double alpha(dt_/(dt_+t_const_));
    double xdot_update(xdot - xdot_est_);
    double ydot_update(ydot - ydot_est_);
    
    if(fabs(xdot_update) > xdot_limit_){ xdot_update = 0.; }
    if(fabs(ydot_update) > ydot_limit_){ ydot_update = 0.; }

    xdot_est_ = xdot_est_ + alpha*xdot_update;
    ydot_est_ = ydot_est_ + alpha*ydot_update;
}

void SimpleAverageEstimator::Output(double & xdot, double & ydot){
    xdot = xdot_est_;
    ydot = ydot_est_;
}

void SimpleAverageEstimator::_SettingParameter(){
    ParamHandler handler(MercuryConfigPath"ESTIMATOR_simple_average.yaml");

    handler.getValue("t_const",t_const_);
    handler.getValue("xdot_limit", xdot_limit_);
    handler.getValue("ydot_limit", ydot_limit_);
}

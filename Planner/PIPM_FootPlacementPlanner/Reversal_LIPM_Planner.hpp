#ifndef VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM
#define VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM

#include <Planner/Planner.hpp>
#include <vector>

class ParamReversalPL{
public:
  double swing_time;
  dynacore::Vect2 des_loc;
  dynacore::Vect3 stance_foot_loc;
  bool b_positive_sidestep;
  dynacore::Vect2 des_vel;
};

class OutputReversalPL{
public:
  double time_modification;
  double switching_state[4];
};

class Reversal_LIPM_Planner: public Planner{
public:
  Reversal_LIPM_Planner();
  virtual ~Reversal_LIPM_Planner();

  virtual void PlannerInitialization(const std::string & setting_file);
  
  virtual void getNextFootLocation(const dynacore::Vect3 & com_pos,
                                   const dynacore::Vect3 & com_vel,
                                   dynacore::Vect3 & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL);

  // Set Functions
  void setOmega(double com_height){
    b_set_omega_ = true;
    omega_ = sqrt(9.81/com_height);
  }
  void CheckEigenValues(double swing_time);

protected:
  // current com state: (x, y, xdot, ydot) : 4
  // switching com state: (x, y, xdot, ydot) : 4
  // target foot: (x, y) : 2
  // swing time: t : 1
  dynacore::Vector planner_save_data_;
  
  std::vector<double> t_prime_;
  std::vector<double> kappa_;
  std::vector<double> x_step_length_limit_;
  std::vector<double> y_step_length_limit_;

  std::vector<double> com_vel_limit_;

  double omega_;
  bool b_set_omega_;

  void _computeSwitchingState(double swing_time,
                              const dynacore::Vect3& com_pos,
                              const dynacore::Vect3& com_vel,
                              const dynacore::Vect3& stance_foot_loc,
                              std::vector<dynacore::Vect2> & switching_state);
  void _StepLengthCheck(dynacore::Vect3 & target_loc,
                        const std::vector<dynacore::Vect2> & switching_state);
  void _StepLengthCheck(dynacore::Vect3 & target_loc,
                        bool b_positive_sidestep,
                        const dynacore::Vect3 & stance_foot);

  int _check_switch_velocity(const std::vector<dynacore::Vect2> & switch_state);

};

#endif

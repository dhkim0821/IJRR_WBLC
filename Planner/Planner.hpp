#ifndef LOCOMOTION_PLANNER
#define LOCOMOTION_PLANNER

#include <Utils/wrap_eigen.hpp>
#include <string>

class Planner{
public:
  Planner(){}
  virtual ~Planner(){}

  virtual void PlannerInitialization(const std::string & _setting_file) = 0;
  virtual void getNextFootLocation(const dynacore::Vect3 & com_pos,
                                   const dynacore::Vect3 & com_vel,
                                   dynacore::Vect3 & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL) = 0;
};

#endif

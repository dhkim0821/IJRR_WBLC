#ifndef DRACO_BIPED_DOUBLE_CONTACT
#define DRACO_BIPED_DOUBLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>

class RobotSystem;
class DracoBip_StateProvider;

class DoubleContact: public WBDC_ContactSpec{
public:
  DoubleContact(const RobotSystem*);
  virtual ~DoubleContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double toe, double heel, double mu, dynacore::Matrix & U);

  const RobotSystem* robot_sys_;
  DracoBip_StateProvider* sp_;
};


#endif

#ifndef DRACO_BIPED_FIXED_BODY_CONTACT
#define DRACO_BIPED_FIXED_BODY_CONTACT

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class DracoBip_StateProvider;

class FixedBodyContact: public WBLC_ContactSpec{
public:
  FixedBodyContact(RobotSystem* );
  virtual ~FixedBodyContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  RobotSystem* robot_sys_;
  DracoBip_StateProvider* sp_;
};

#endif

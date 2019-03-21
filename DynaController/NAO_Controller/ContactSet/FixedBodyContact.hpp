#ifndef NAO_FIXED_BODY_CONTACT
#define NAO_FIXED_BODY_CONTACT

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class NAO_StateProvider;

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
  NAO_StateProvider* sp_;
};

#endif

#ifndef MERCURY_FIXED_BODY_CONTACT
#define MERCURY_FIXED_BODY_CONTACT

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class Mercury_StateProvider;

class FixedBodyContact: public WBLC_ContactSpec{
public:
  FixedBodyContact(const RobotSystem* );
  virtual ~FixedBodyContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const RobotSystem* robot_sys_;
  Mercury_StateProvider* sp_;
};

#endif

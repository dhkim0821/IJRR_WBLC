#ifndef VALKYRIE_FIXED_BODY_CONTACT
#define VALKYRIE_FIXED_BODY_CONTACT

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class Valkyrie_StateProvider;

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
  Valkyrie_StateProvider* sp_;
};
#endif

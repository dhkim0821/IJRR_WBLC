#ifndef VALKYRIE_SINGLE_CONTACT
#define VALKYRIE_SINGLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class Valkyrie_StateProvider;

class SingleContact: public WBDC_ContactSpec{
public:
  SingleContact(const RobotSystem* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(double max_fz){ max_Fz_ = max_fz; }
protected:
  double max_Fz_;
  int dim_U_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double x, double y, double mu, dynacore::Matrix & U);
  const RobotSystem* robot_sys_;
  Valkyrie_StateProvider* sp_;

  int contact_pt_;
};

#endif

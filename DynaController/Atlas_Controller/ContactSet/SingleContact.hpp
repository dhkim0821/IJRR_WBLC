#ifndef Atlas_SINGLE_CONTACT
#define Atlas_SINGLE_CONTACT

#include <WBLC/WBLC_ContactSpec.hpp>
class RobotSystem;
class Atlas_StateProvider;

class SingleContact: public WBLC_ContactSpec{
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
  Atlas_StateProvider* sp_;

  int contact_pt_;
};

#endif

#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include <Utils/wrap_eigen.hpp>

class ContactSpec{
public:
  ContactSpec(int dim):dim_contact_(dim), b_set_contact_(false){}
  virtual ~ContactSpec(){}

  void getContactJacobian(dynacore::Matrix & Jc){ Jc = Jc_; }
  void getJcDotQdot(dynacore::Vector & JcDotQdot) { JcDotQdot = JcDotQdot_; }
  int getDim(){ return dim_contact_; }
  void UnsetContact(){ b_set_contact_ = false; }

  bool UpdateContactSpec(){
    _UpdateJc();
    _UpdateJcDotQdot();
    _AdditionalUpdate();
    b_set_contact_ = true;
    return true;
  }

protected:
  virtual bool _UpdateJc() = 0;
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _AdditionalUpdate() = 0;

  dynacore::Matrix Jc_;
  dynacore::Vector JcDotQdot_;
  int dim_contact_;
  bool b_set_contact_;
};
#endif

#ifndef SRNAO
#define SRNAO

#include <srSysGenerator/SystemGenerator.h>

class srNao: public SystemGenerator {
 public:
  srNao();
  virtual ~srNao();

 private:
  void _SetCollision();
  void _SetInitialConf();
  void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif

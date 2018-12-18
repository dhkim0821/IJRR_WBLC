#ifndef DRACO_H
#define DRACO_H

#include <srSysGenerator/SystemGenerator.h>

class srDraco: public SystemGenerator {
 public:
  srDraco();
  virtual ~srDraco();

 private:
  void _SetCollision();
  void _SetInitialConf();
  void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif

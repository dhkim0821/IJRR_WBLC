#ifndef ATLAS_H
#define ATLAS_H

#include <srSysGenerator/SystemGenerator.h>

class Atlas: public SystemGenerator {
 public:
  Atlas();
  virtual ~Atlas();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif

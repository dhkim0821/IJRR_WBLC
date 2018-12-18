#ifndef Quadruped_H
#define Quadruped_H

#include <srSysGenerator/SystemGenerator.h>

class Quadruped: public SystemGenerator {
 public:
  Quadruped();
  virtual ~Quadruped();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif

#ifndef SagitP3_H
#define SagitP3_H

#include <srSysGenerator/SystemGenerator.h>

class SagitP3: public SystemGenerator {
 public:
  SagitP3();
  virtual ~SagitP3();

 protected:
  double hanging_height_;
  double collision_offset_;
  int initial_posture_;

  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;

};

#endif

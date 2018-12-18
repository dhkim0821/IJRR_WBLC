#ifndef Cassie_H
#define Cassie_H

#include <srSysGenerator/SystemGenerator.h>

class Cassie: public SystemGenerator {
 public:
  Cassie();
  virtual ~Cassie();

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

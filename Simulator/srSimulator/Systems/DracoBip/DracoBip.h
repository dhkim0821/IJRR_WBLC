#ifndef DracoBip_H
#define DracoBip_H

#include <srSysGenerator/SystemGenerator.h>

class DracoBip: public SystemGenerator {
 public:
  DracoBip();
  virtual ~DracoBip();

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

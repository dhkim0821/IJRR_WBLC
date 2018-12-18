#ifndef DRACO_H
#define DRACO_H

#include <srSysGenerator/SystemGenerator.h>

class srDraco: public SystemGenerator {
 public:
  srDraco(Vec3 location);
  virtual ~srDraco();

  std::vector<srCollision*> collision_;
 private:
  void _SetCollision();
  void _SetInitialConf();
  void _SetJointLimit();
  void _SetInertia();

  void _AssembleRobot(Vec3 location);
  void _DefineLinks();
  void _DefineVirtualJoint();
  void _DefineActuatedJoint();
};

#endif

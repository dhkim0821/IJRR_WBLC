#ifndef _MERCURY_
#define _MERCURY_

#include "srDyn/srSpace.h"
#include "Ground.h"
#include <vector>
#include <srSysGenerator/SystemGenerator.h>

/* class Mercury: public srSystem */
class Mercury: public SystemGenerator
{
public:
  Mercury(const Vec3 & location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
  virtual ~Mercury();
  void SetConfiguration(const std::vector<double>& _conf);

protected:
  int initial_posture_;
  double hanging_height_;
  std::vector<srCollision*> collision_;

  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

};

#endif

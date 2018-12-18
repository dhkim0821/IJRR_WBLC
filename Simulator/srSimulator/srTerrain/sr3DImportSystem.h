#ifndef SR_IMPORT
#define SR_IMPORT

#include "srDyn/srSpace.h"

#include <stdio.h>
#include <iostream>
#include <vector>

class sr3DImportSystem: public srSystem
{
public:
  sr3DImportSystem(string modelnamepath,
                   const Vec3 & location,
                   const Vec3 & orientation,
                   srJoint::ACTTYPE joint_type);

  virtual ~sr3DImportSystem();

  std::vector<srLink*> m_Link;
  std::vector<srLink*> m_VLink;
  std::vector<srPrismaticJoint*> m_VPjoint;
  std::vector<srRevoluteJoint*> m_VRjoint;
  std::vector<srCollision*>	m_Collision;

  void SetColor(float r, float g, float b, float alpha = 1.0f);

protected:
  void _BuildTerrain();

  void _Set_Stair_Terrain();
  void _Set_Virtual_Link_Joint();
  void _Set_Link_Shape();
  void _SetCollision();
  void _Set_Initial_Conf();

  srJoint::ACTTYPE joint_type_;
  const char* modelnamepath_;
  float R, G, B;
  Vec3 ori_;
};



#endif

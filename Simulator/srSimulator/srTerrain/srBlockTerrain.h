#ifndef SR_BLOCK_TERRAIN
#define SR_BLOCK_TERRAIN

#include "srDyn/srSpace.h"

#include <stdio.h>
#include <iostream>
#include <vector>

class srBlockTerrain: public srSystem
{
public:
  srBlockTerrain(int num_block,
               const Vec3 & location);

  virtual ~srBlockTerrain();

  void SetColor(float r, float g, float b, float alpha = 1.0f);

protected:
  void _BuildTerrain(const std::vector<double> & xpos_list,
                     const std::vector<double> & ypos_list,
                     const std::vector<double> & zpos_list,
                     const std::vector<double> & l_list,
                     const std::vector<double> & w_list,
                     const std::vector<double> & h_list,
                     const std::vector<Vec3> & ori_list = std::vector<Vec3>(0));

  int num_blocks_;
  srLink m_BaseLink;
  std::vector<srLink*>      m_TLink;
  std::vector<srWeldJoint*>      m_Tjoint;
  std::vector<srCollision*>		m_Collision;

  void _Set_Stair_Terrain();
  void _Set_Link_Shape(const std::vector<double> & l_list,
                       const std::vector<double> & w_list,
                       const std::vector<double> & h_list);

  void _ConnectLinks(const std::vector<double> & xpos_list,
                     const std::vector<double> & ypos_list,
                     const std::vector<double> & zpos_list,
                     const std::vector<Vec3> & ori_list);

  void _SetCollision();

  float R, G, B;
};



#endif

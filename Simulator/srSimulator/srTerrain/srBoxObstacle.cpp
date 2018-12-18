#include "srBoxObstacle.h"

 srBoxObstacle::srBoxObstacle(sejong::Vect3 init_pos, sejong::Vect3 lwh_):
   srBlockTerrain(1, Vec3(0,0,0)){

  std::vector<double> xpos_list(1,0.);
  std::vector<double> ypos_list(1,0.);
  std::vector<double> zpos_list(1,0.);

  std::vector<double> l_list(1,0.);
  std::vector<double> w_list(1,0.);
  std::vector<double> h_list(1,0.);

  xpos_list[0] = init_pos[0];
  ypos_list[0] = init_pos[1];
  zpos_list[0] = init_pos[2];

  w_list[0] = lwh_[1];
  l_list[0] = lwh_[0];
  h_list[0] = lwh_[2];

 // R = (100)*0.9f;
 // G = (60)*0.005f;
 // B = (30)*0.005f;

 R = (0);
 G = (0);
 B = (1);

  _BuildTerrain(xpos_list, ypos_list, zpos_list,
                l_list, w_list, h_list);
}
srBoxObstacle::~srBoxObstacle(){

}

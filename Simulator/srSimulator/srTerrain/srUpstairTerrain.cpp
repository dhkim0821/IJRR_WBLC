#include "srUpstairTerrain.h"

srUpstairTerrain::srUpstairTerrain(int num_stairs, const Vec3 &location):
  srBlockTerrain(num_stairs, location){

  std::vector<double> xpos_list(num_stairs,0.);
  std::vector<double> ypos_list(num_stairs,0.);
  std::vector<double> zpos_list(num_stairs,0.);

  std::vector<double> l_list(num_stairs,0.);
  std::vector<double> w_list(num_stairs,0.);
  std::vector<double> h_list(num_stairs,0.);

  for (int i(0); i< num_stairs; ++i){
    xpos_list[i] = 0.3 * i;
    ypos_list[i] = 0.0;
    zpos_list[i] = 0.06 * i;

    l_list[i] = 0.3;
    w_list[i] = 0.7;
    h_list[i] = 0.03;
  }

  _BuildTerrain(xpos_list, ypos_list, zpos_list,
                l_list, w_list, h_list);
}
srUpstairTerrain::~srUpstairTerrain(){

}

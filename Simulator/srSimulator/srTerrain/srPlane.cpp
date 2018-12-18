#include "srPlane.h"

srPlane::srPlane(double slope, const Vec3 &location):
  srBlockTerrain(1, location){

  std::vector<double> xpos_list(1,3.);
  std::vector<double> ypos_list(1,0.);
  std::vector<double> zpos_list(1, 3. * tan(slope) - 0.025);

  std::vector<double> l_list(1, 6.);
  std::vector<double> w_list(1, 6.);
  std::vector<double> h_list(1, 0.05);

  std::vector<Vec3> ori_list(1);

  ori_list[0][1] = slope;

  _BuildTerrain(xpos_list, ypos_list, zpos_list,
                l_list, w_list, h_list, ori_list);
}
srPlane::~srPlane(){

}

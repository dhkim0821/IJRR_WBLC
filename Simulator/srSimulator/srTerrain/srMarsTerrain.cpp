#include "srMarsTerrain.h"

#define NUM_PIECES 40

srMarsTerrain::srMarsTerrain(const Vec3 &location):
  srBlockTerrain(NUM_PIECES, location){

  std::vector<double> xpos_list(NUM_PIECES,0.);
  std::vector<double> ypos_list(NUM_PIECES,0.);
  std::vector<double> zpos_list(NUM_PIECES,0.);
  std::vector<Vec3> ori_list(NUM_PIECES);

  std::vector<double> l_list(NUM_PIECES,0.);
  std::vector<double> w_list(NUM_PIECES,0.);
  std::vector<double> h_list(NUM_PIECES,0.);

  int forward(-1);
  int num_per_line(4);
  for (int i(0); i< NUM_PIECES; ++i){
    if(i%num_per_line == 0) ++forward ;

    xpos_list[i] = 0.35 * forward;
    ypos_list[i] = -0.6 + 0.4 * (i%num_per_line);
    zpos_list[i] = SR_RAND(0.05, 0.3);
    ori_list[i] = Vec3(SR_RAND(-0.05, 0.05), SR_RAND(-0.3, 0.3), SR_RAND(-0.15, 0.15));
    // ori_list[i] = Vec3();

    l_list[i] = SR_RAND(0.23, 0.4);
    w_list[i] = 0.4;
    h_list[i] = 0.08;
  }

  _BuildTerrain(xpos_list, ypos_list, zpos_list, 
                l_list, w_list, h_list,
                ori_list);
}
srMarsTerrain::~srMarsTerrain(){
  
}


// double l_[20]={0.3,0.2,0.4,0.3,0.25,
//                0.2,0.4,0.2,0.35,0.3,
//                0.2,0.3,0.25,0.32,0.25,
//                0.3,0.23,0.21,0.33,0.2};
    
// double w_[20]={0.35,0.35,0.35,0.35,0.35,
//                0.35,0.35,0.35,0.35,0.35,
//                0.35,0.35,0.35,0.35,0.35,
//                0.35,0.35,0.35,0.35,0.35};
    
// double h_[20]={0.12,0.15,0.13,0.11,0.06,
//                0.08,0.15,0.13,0.10,0.16,
//                0.13,0.15,0.13,0.11,0.26,
//                0.05,0.10,0.11,0.16,0.11};
// Vec3 ori_[20]={Vec3(0,0,0.3),Vec3(0,-0.12,-0.2),Vec3(0,-0.03,0.1),Vec3(0,0.05,0.26),Vec3(0,0.01,0.05),
//                Vec3(0,0,0.3),Vec3(0,-0.02,-0.3),Vec3(0,-0.03,0.1),Vec3(0,0.05,0.2),Vec3(0,0.03,-0.01),
//                Vec3(0,0,0.3),Vec3(0,-0.02,-0.5),Vec3(0,-0.03,0.1),Vec3(0,0.15,0.26),Vec3(0,0.1,-0.14),
//                Vec3(0,-0.02,-0.16),Vec3(0,-0.23,0.1),Vec3(0,0.05,0.16),Vec3(0,0.1,0.04),Vec3(0,0.13,-0.23)};    

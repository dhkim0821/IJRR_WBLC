#include "srRoom.h"
#include <cmath>

srRoom::srRoom(const Vec3 &location):
  srBlockTerrain(5, location){

  std::vector<double> xpos_list(5,0.);
  std::vector<double> ypos_list(5,0.);
  std::vector<double> zpos_list(5,0.);

  std::vector<double> l_list(5,0.);
  std::vector<double> w_list(5,0.);
  std::vector<double> h_list(5,0.);
  std::vector<Vec3> ori_list(5);

  for (int i(0); i< 5; ++i){
    zpos_list[i] = wall_height * 0.5;
    w_list[i] = wall_width;
    h_list[i] = wall_height;
  }
  //6->3->12(r)->12(l)->9
  xpos_list[0] = room_x_min;
  xpos_list[1] = (room_x_max + room_x_min) * 0.5;
  xpos_list[2] = room_x_max;
  xpos_list[3] = room_x_max;
  xpos_list[4] = (room_x_max + room_x_min) * 0.5;

  ypos_list[0] = (room_y_max + room_y_min) * 0.5;
  ypos_list[1] = room_y_max;
  ypos_list[2] = (room_y_max + door_y_max) * 0.5;
  ypos_list[3] = (room_y_min + door_y_min) * 0.5;
  ypos_list[4] = room_y_min;

  l_list[0] = room_y_max - room_y_min;
  l_list[1] = room_x_max - room_x_min;
  l_list[2] = room_y_max - door_y_max;
  l_list[3] = door_y_min - room_y_min;
  l_list[4] = room_x_max - room_x_min;

  ori_list[0] = Vec3(0.5 * M_PI, 0., 0.);
  ori_list[1] = Vec3(0., 0., 0.);
  ori_list[2] = Vec3(-0.5 * M_PI, 0., 0.);
  ori_list[3] = Vec3(-0.5 * M_PI, 0., 0.);
  ori_list[4] = Vec3(-M_PI, 0., 0.);

  _BuildTerrain(xpos_list, ypos_list, zpos_list,
                l_list, w_list, h_list, ori_list);
}
srRoom::~srRoom(){

}

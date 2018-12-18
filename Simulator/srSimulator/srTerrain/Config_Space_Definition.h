#ifndef CONFIG_SPACE_DEF
#define CONFIG_SPACE_DEF

//ROOM(DOOR, WALL)

#define room_x_min -2.
//#define room_x_min -1.5
#define room_x_max 12.
#define room_y_min -16.
#define room_y_max 2.
#define wall_height 2.
#define wall_width 0.05
#define door_y_min -15.
#define door_y_max -14.

// #define obs_height 1.

//DYNAMIC OBSTACLE
#define obs_1_x_point_1 3.
#define obs_1_y_point_1 5.
#define obs_1_x_point_2 3.
#define obs_1_y_point_2 0.5.
#define obs_1_width 2.
#define obs_1_length 2.
#define obs_1_height 1.
#define obs_1_vel 0.1



//cart 1
//moving (3,5) <--> (3,0.5) with velocity 0.1 m/s
#define obs_2_x_point_1 3.
#define obs_2_y_point_1 5.
#define obs_2_x_point_2 3.
#define obs_2_y_point_2 0.5.
#define obs_2_width 2.
#define obs_2_length 2.
#define obs_2_height 1.
#define obs_2_vel 0.1

//cart 2
//moving (5.5,-2) <--> (8,-2) with velocity 0.1m/s
#define obs_3_x_point_1 5.5
#define obs_3_y_point_1 -2.
#define obs_3_x_point_2 8.
#define obs_3_y_point_2 -2.
#define obs_3_width 2.
#define obs_3_length 2.
#define obs_3_height 1.
#define obs_3_vel 0.1

//rotating arm(3, -6), rad = 2, angvel = pi/6
#define obs_4_x 3.
#define obs_4_y -6.
#define obs_4_rad 2.
#define obs_4_angvel 0.52

#endif

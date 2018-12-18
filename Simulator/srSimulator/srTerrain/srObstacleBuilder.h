// #ifndef SR_OBS_BUILD
// #define SR_OBS_BUILD
//
// #include "LocomotionPlanner/EnvironmentSetup/Obstacle.h"
// #include "utils/wrap_eigen.hpp"
// #include "utils/utilities.h"
//
//
// class srObstacleBuilder{
// public:
//   srObstacleBuilder();
//   ~srObstacleBuilder();
//   std::vector<Obstacle> get_obstacle_list();
//   void save_obs_vectors();
//   void save_obs_vectors(double sim_time, double step_size);
//
// private:
//   std::vector<Obstacle> obs_list_;
//
//   void add_kiva_bots();
//   void add_table();
//   void add_rot_arm();
//
//   sejong::Vect3 init_pos, lwh;
//   MovementParam mov_param_1, mov_param_2, mov_param_3;
//   Obstacle obs;
// };
//
//
// #endif

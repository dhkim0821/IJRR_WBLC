// #include "srObstacleBuilder.h"
//
//
//
// srObstacleBuilder::srObstacleBuilder()
// {
//   add_kiva_bots();
//   add_table();
//   add_rot_arm();
// }
//
// srObstacleBuilder::~srObstacleBuilder()
// {}
//
// std::vector<Obstacle> srObstacleBuilder::get_obstacle_list()
// {
//   return obs_list_;
// }
//
//
// void srObstacleBuilder::save_obs_vectors()
// {
//   save_obs_vectors(20.,0.05);
// }
//
// void srObstacleBuilder::save_obs_vectors(double sim_time, double step_size)
// {
//   sejong::Transform tf;
//
//   sejong::Vector corners(9);
//   sejong::Vect3 one_corner(3);
//
//   corners.setConstant(obs_list_.size());// First line should tell matlab how many obstacles to read per frame
//   sejong::saveVector(corners, "obs_anim");  // First line should tell matlab how many obstacles to read per frame
//
//   for(double t = 0.; t < sim_time; t += step_size)
//   {
//     for(std::vector<Obstacle>::iterator it = obs_list_.begin(); it < obs_list_.end(); ++it)
//     {
//       it->get_transform(t, tf);
//       corners[0] = t;
//
//       it->get_init_corner(0, one_corner);
//       one_corner = tf * one_corner;
//       corners[1] = one_corner[0];
//       corners[2] = one_corner[1];
//
//       it->get_init_corner(1, one_corner);
//       one_corner = tf * one_corner;
//       corners[3] = one_corner[0];
//       corners[4] = one_corner[1];
//
//       it->get_init_corner(2, one_corner);
//       one_corner = tf * one_corner;
//       corners[5] = one_corner[0];
//       corners[6] = one_corner[1];
//
//       it->get_init_corner(3, one_corner);
//       one_corner = tf * one_corner;
//       corners[7] = one_corner[0];
//       corners[8] = one_corner[1];
//
//       sejong::saveVector(corners, "obs_anim");
//     }
//   }
// }
//
// void srObstacleBuilder::add_kiva_bots()
// {
//   double amp(0.5);
//   double freq(8.*M_PI/10.);
//   double phase_lag(M_PI/10.);
//
//   init_pos << -8., 6., 0.;
//   lwh      << 3., 1., 1.;
//   obs = Obstacle( init_pos, lwh );
//   mov_param_1 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::LINEAR,
//                               sejong::Vect3::Zero(), amp*freq, 0. );
//   mov_param_2 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::SINUSOID,
//                               sejong::Vect3::Zero(), amp, freq , phase_lag);
//   obs.add_movement(mov_param_1);
//   obs.add_movement(mov_param_2);
//   obs_list_.push_back(obs);
//
//   init_pos << -4. ,6., 0.;
//   lwh      <<  3., 1., 1.;
//   obs = Obstacle( init_pos, lwh );
//   obs.add_movement(mov_param_1);
//   mov_param_2.offset_ += phase_lag;
//   obs.add_movement(mov_param_2);
//   obs_list_.push_back(obs);
//
//   init_pos <<  0. ,6., 0.;
//   lwh      <<  3., 1., 1.;
//   obs = Obstacle( init_pos, lwh );
//   obs.add_movement(mov_param_1);
//   mov_param_2.offset_ += phase_lag;
//   obs.add_movement(mov_param_2);
//   obs_list_.push_back(obs);
//
//   init_pos << -12. ,6., 0.;
//   lwh      <<   3., 1., 1.;
//   obs = Obstacle( init_pos, lwh );
//   obs.add_movement(mov_param_1);
//   mov_param_2.offset_ = 0;
//   obs.add_movement(mov_param_2);
//   obs_list_.push_back(obs);
// }
//
// void srObstacleBuilder::add_table()
// {
//   init_pos << -3.,-2., 0.;
//   lwh      <<  3., 6., 1.;
//   obs = Obstacle( init_pos, lwh );
//   obs_list_.push_back(obs);
// }
//
// void srObstacleBuilder::add_rot_arm()
// {
//   init_pos << 0.,-8.25, 0.;
//   lwh      << 6., 0.50, 1.;
//   obs = Obstacle( init_pos, lwh );
//
//   mov_param_1 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::SINUSOID,
//                               sejong::Vect3(lwh[0],lwh[1]/2.,0.) , -M_PI/8., M_PI/10. );
//   obs.add_movement(mov_param_1);
//   obs_list_.push_back(obs);
// }

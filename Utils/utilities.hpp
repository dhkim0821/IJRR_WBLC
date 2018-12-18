#ifndef UTILITIES
#define UTILITIES

#include "wrap_eigen.hpp"
#include <iostream>

#define SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }

namespace dynacore{
  static std::list< std::string > gs_fileName_string; //global & static

  // Box Muller Transform
  double generator_white_noise(double mean, double var);
  double generator_truncated_white_noise(double mean, double var, double min, double max);
  // min, var > 0
  // mean = a * b
  // variance = a * b * b
  double generator_gamma_noise(double a, double b);

  std::string pretty_string(double vv);
  std::string pretty_string(dynacore::Vector const & vv);
  std::string pretty_string(dynacore::Quaternion const & qq);
  std::string pretty_string(dynacore::Matrix const & mm, std::string const & prefix);
  std::string pretty_string_short(double vv);


  void pretty_print(dynacore::Vector const & vv, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix="", bool nonl = false);
  void pretty_print(dynacore::Vect3 const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix="", bool nonl = false);
  void pretty_print(const std::vector<double> & _vec, const char* title);
  void pretty_print(const std::vector<int> & _vec, const char* title);
  void pretty_print(const double * _vec, const char* title, int size);
  void pretty_print(dynacore::Quaternion const & qq, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix="", bool nonl = false);
  void pretty_print(dynacore::Matrix const & mm, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix ="",
                    bool vecmode = false, bool nonl = false);
  void pretty_print_short(dynacore::Matrix const & mm, std::ostream & os,
                          std::string const & title,
                          std::string const & prefix ="",
                          bool vecmode = false, bool nonl = false);

  void printVectorSequence(const std::vector<dynacore::Vector> & seq, std::string _name);

  // Save Data
  void saveVectorSequence(const std::vector<dynacore::Vector> & seq,
                          std::string _name, bool b_param = false);
  void saveVector(const dynacore::Vector & _vec, std::string _name, bool b_param = false);
  void saveVector(const std::vector<double> & _vec, std::string _name, bool b_param = false);
  void saveVector(double * _vec, std::string _name, int size, bool b_param = false);
  void saveQuaternion(const dynacore::Quaternion & qq, std::string _name, bool b_param = false);
  void saveValue(double _value, std::string _name, bool b_param = false);
  void cleaning_file(std::string _file_name, std::string & ret_file, bool b_param);

  // Smooth Changing
  double smooth_changing(double ini, double end,
                         double moving_duration, double curr_time);
  double smooth_changing_vel(double ini, double end,
                             double moving_duration, double curr_time);
  double smooth_changing_acc(double ini, double end,
                             double moving_duration, double curr_time);

  double MinMaxBound(double value, double min, double max);
  bool MinMaxCheck(double value, double min, double max, bool b_exit = false);
  void sqrtm(const dynacore::Matrix & mt, dynacore::Matrix & sqrt_mt);

  //jh
  void read_file(std::string file_name_, std::vector<std::string> & _vec);
  void split_string(std::string* str_array, std::string strTarget, std::string strTok );
  dynacore::Matrix crossmat(dynacore::Vect3);
  //jh
}

#endif

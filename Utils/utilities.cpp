#include "utilities.hpp"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Eigen/Eigenvalues>
#include <Configuration.h>
#include <string>

using namespace std;

namespace dynacore{
  std::string pretty_string(dynacore::Vector const & vv)
  {
    ostringstream os;
    pretty_print(vv, os, "", "", true);
    return os.str();
  }


  std::string pretty_string(dynacore::Quaternion const & qq)
  {
    ostringstream os;
    pretty_print(qq, os, "", "", true);
    return os.str();
  }


  std::string pretty_string(dynacore::Matrix const & mm, std::string const & prefix)
  {
    ostringstream os;
    pretty_print(mm, os, "", prefix);
    return os.str();
  }


  void pretty_print(dynacore::Vector const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool nonl)
  {
    pretty_print((dynacore::Matrix const &) vv, os, title, prefix, true, nonl);
  }

  void pretty_print(dynacore::Vect3 const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix, bool nonl){
    pretty_print((dynacore::Matrix const &) vv, os, title, prefix, true, nonl);
  }


  void pretty_print(dynacore::Quaternion const & qq, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool nonl)
  {
    pretty_print(qq.coeffs(), os, title, prefix, true, nonl);
  }


  std::string pretty_string(double vv)
  {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    /*#ifndef WIN32
      if (isinf(vv)) {
      snprintf(buf, buflen-1, " inf    ");
      }
      else if (isnan(vv)) {
      snprintf(buf, buflen-1, " nan    ");
      }
      else if (fabs(fmod(vv, 1)) < 1e-9) {
      snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(vv)));
      }
      else {
      snprintf(buf, buflen-1, "% 6.6f  ", vv);
      }
      #else // WIN32*/
    snprintf(buf, buflen-1, "% 6.6f  ", vv);
    //#endif // WIN32
    string str(buf);
    return str;
  }
  std::string pretty_string_short(double vv)
  {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    snprintf(buf, buflen-1, "% 6.3f  ", vv);
    string str(buf);
    return str;
  }

  void pretty_print(const std::vector<double> & _vec, const char* title){
    printf("%s: ", title);
    for( int i(0); i< _vec.size(); ++i){
      printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
  }
  void pretty_print(const std::vector<int> & _vec, const char* title){
    printf("%s: ", title);
    for( int i(0); i< _vec.size(); ++i){
      printf("%d, \t", _vec[i]);
    }
    printf("\n");
  }

  void pretty_print(const double * _vec, const char* title, int size){
    printf("%s: ", title);
    for(int i(0); i< size; ++i){
      printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
  }
  void pretty_print(dynacore::Matrix const & mm, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool vecmode, bool nonl)
  {
    char const * nlornot("\n");
    if (nonl) {
      nlornot = "";
    }
    if ( ! title.empty()) {
      os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
      os << prefix << " (empty)" << nlornot;
    }
    else {
      // if (mm.cols() == 1) {
      //   vecmode = true;
      // }

      if (vecmode) {
        if ( ! prefix.empty())
          os << prefix;
        for (int ir(0); ir < mm.rows(); ++ir) {
          os << pretty_string(mm.coeff(ir, 0));
        }
        os << nlornot;

      }
      else {

        for (int ir(0); ir < mm.rows(); ++ir) {
          if ( ! prefix.empty())
            os << prefix;
          for (int ic(0); ic < mm.cols(); ++ic) {
            os << pretty_string(mm.coeff(ir, ic));
          }
          os << nlornot;
        }

      }
    }
  }

  void pretty_print_short(dynacore::Matrix const & mm, std::ostream & os,
                          std::string const & title, std::string const & prefix,
                          bool vecmode, bool nonl)
  {
    char const * nlornot("\n");
    if (nonl) {
      nlornot = "";
    }
    if ( ! title.empty()) {
      os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
      os << prefix << " (empty)" << nlornot;
    }
    else {
      if (vecmode) {
        if ( ! prefix.empty())
          os << prefix;
        for (int ir(0); ir < mm.rows(); ++ir) {
          os << pretty_string_short(mm.coeff(ir, 0));
        }
        os << nlornot;
      }
      else {

        for (int ir(0); ir < mm.rows(); ++ir) {
          if ( ! prefix.empty())
            os << prefix;
          for (int ic(0); ic < mm.cols(); ++ic) {
            os << pretty_string_short(mm.coeff(ir, ic));
          }
          os << nlornot;
        }
      }
    }
  }


  ////////////////////////////////////////////////////
  //           Save Data
  ////////////////////////////////////////////////////
  void saveVector(const dynacore::Vector & _vec, string _name, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);

    std::ofstream savefile(file_name.c_str(), ios::app);
    for (int i(0); i < _vec.rows(); ++i){
      savefile<<_vec(i)<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
  }

  void saveVector(double * _vec, std::string _name, int size, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), ios::app);

    for (int i(0); i < size; ++i){
      savefile<<_vec[i]<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
  }

  void saveVector(const std::vector<double> & _vec, string _name, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), ios::app);
    for (int i(0); i < _vec.size(); ++i){
      savefile<<_vec[i]<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
  }
  void saveQuaternion(const dynacore::Quaternion & qq, std::string _name, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), ios::app);
    savefile<<qq.w()<< "\t";
    savefile<<qq.x()<< "\t";
    savefile<<qq.y()<< "\t";
    savefile<<qq.z()<< "\t";
    savefile<<"\n";
    savefile.flush();
  }


  void saveValue(double _value, string _name, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), ios::app);

    savefile<<_value <<"\n";
    savefile.flush();
  }

  void saveVectorSequence(const std::vector<dynacore::Vector> & seq, string _name, bool b_param){
    string file_name;
    cleaning_file(_name, file_name, b_param);

    std::ofstream savefile(file_name.c_str(), ios::app);

    for (int i(0); i< seq.size(); ++i){
      for (int j(0); j< seq[i].rows(); ++j){
        savefile<<seq[i](j)<<"\t";
      }
      savefile<<"\n";
    }
    savefile.flush();
  }

  void cleaning_file(string  _file_name, string & _ret_file, bool b_param){
    if(b_param)
      _ret_file += THIS_COM"parameter_data/";
    else
      _ret_file += THIS_COM"experiment_data/";

    _ret_file += _file_name;
    _ret_file += ".txt";

    std::list<string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if(gs_fileName_string.end() == iter){
      gs_fileName_string.push_back(_file_name);
      remove(_ret_file.c_str());
    }
  }
  ////////////////////////////////////////////////////
  //           End of Save Data
  ////////////////////////////////////////////////////

  void printVectorSequence(const std::vector<dynacore::Vector> & seq, string _name){
    std::cout<<_name<<":\n";
    for (int i(0); i< seq.size(); ++i){
      for (int j(0); j< seq[i].rows(); ++j){
        printf("% 6.4f, \t", seq[i][j]);
      }
      std::cout<<"\n";
    }
  }

  double generator_truncated_white_noise(double mean, double var, double min, double max){
    double ret;
    do{
      ret = generator_white_noise(mean, var);
    } while (ret <min || max < ret );

    return ret;
  }

  double generator_white_noise(double mean, double var){

    static bool hasSpare = false;
    static double rand1, rand2;

    if(hasSpare){
      hasSpare = false;
      return sqrt(var*rand1)*sin(rand2) + mean;
    }
    hasSpare = true;

    rand1 = rand() / ((double ) RAND_MAX);
    if(rand1 < 1e-100) rand1 = 1e-100;
    rand1 = -2*log(rand1);
    rand2 = rand() / ((double ) RAND_MAX) * M_PI * 2.;

    return mean + sqrt(var*rand1)*cos(rand2);
  }
  double generator_gamma_noise(double a, double b){
    /* assume a > 0 */
     if (a < 1)      {
       double u = rand()/((double)RAND_MAX);
       return generator_gamma_noise (1.0 + a, b) * pow (u, 1.0 / a);
     }

     {
       double x, v, u;
       double d = a - 1.0 / 3.0;
       double c = (1.0 / 3.0) / sqrt (d);

       while (true){
         do{
           x = generator_white_noise (0., 1.0);
           v = 1.0 + c * x;
         }
         while (v <= 0);

         v = v * v * v;
         u = rand()/((double)RAND_MAX);

         if (u < 1 - 0.0331 * x * x * x * x)  break;
         if (log (u) < 0.5 * x * x + d * (1 - v + log (v)))  break;
       }
       return b * d * v;
     }
  }

  void sqrtm(const dynacore::Matrix & mt, dynacore::Matrix & sqrt_mt){
    Eigen::EigenSolver<dynacore::Matrix> es(mt, true);
    Eigen::MatrixXcd V = es.eigenvectors();
    Eigen::VectorXcd Dv = es.eigenvalues();
    Eigen::MatrixXcd sqrtD(mt.cols(), mt.rows());
    sqrtD = Dv.cwiseSqrt().asDiagonal();
    Eigen::MatrixXcd tmp_sqrt_mt = V*sqrtD*V.inverse();

    sqrt_mt = tmp_sqrt_mt.real();
  }


  double smooth_changing(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = ini + (end - ini)*0.5*(1-cos(curr_time/moving_duration * M_PI));
    if(curr_time>moving_duration){
      ret = end;
    }
    return ret;
  }

  double smooth_changing_vel(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = (end - ini)*0.5*(M_PI/moving_duration)*sin(curr_time/moving_duration*M_PI);
    if(curr_time>moving_duration){
      ret = 0.0;
    }
    return ret;
  }
  double smooth_changing_acc(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = (end - ini)*0.5*(M_PI/moving_duration)*(M_PI/moving_duration)*cos(curr_time/moving_duration*M_PI);
    if(curr_time>moving_duration){
      ret = 0.0;
    }
    return ret;
  }

  double MinMaxBound(double value, double min, double max){
    if(value > max){
      return max;
    }
    else if (value < min){
      return min;
    }
    return value;
  }
  bool MinMaxCheck(double value, double min, double max, bool b_exit){
    if(value > max){
      if(b_exit) exit(0);
      return true;
    }
    else if (value < min){
      if(b_exit) exit(0);
      return true;
    }
    return false;
  }

  //jh
  void read_file(std::string _file_name, std::vector<std::string> & _vec){
    ifstream  InputFile(_file_name.c_str());
    std::string tempstring;
    if(!InputFile.is_open()){
      cout << "Data file load error... check the data file" << endl;
      exit(0);
    }
    else{
      while(!InputFile.eof()){
        InputFile.clear();
        getline(InputFile,tempstring);
        _vec.push_back(tempstring);
      }
      InputFile.close();
    }
  }

  void split_string(std::string* str_array, std::string strTarget, std::string strTok ){
    int nCutPos = 0;
    int nIndex = 0;
    while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos){
      if (nCutPos > 0){
        str_array[nIndex++] = strTarget.substr(0, nCutPos);
      }
      strTarget = strTarget.substr(nCutPos+1);
    }
    if(strTarget.length() > 0){
      str_array[nIndex++] = strTarget.substr(0, nCutPos);
    }
  }

  dynacore::Matrix crossmat(dynacore::Vect3 r){
      dynacore::Matrix ret(3, 3);
      ret << 0          , -1 * r[2]   , r[1],
             r[2]       ,  0          , -1 * r[0],
             -1 * r[1]  , r[0]        , 0;

      return ret;
  }
  //jh
}


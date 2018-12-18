#ifndef DYNACORE_WRAP_EIGEN
#define DYNACORE_WRAP_EIGEN

#define DYNA_SAFE_DELETE(p)    if(p) { delete (p); (p) = NULL; }
#define DYNA_SAFE_DELETE_AR(p)		if(p) { delete [] p; (p) = NULL; }
#define DYNA_SAFE_RELEASE(p)		if(p) { (p)->Release(); (p) = NULL; }

#define DYNA_ISZERO(x)	(fabs(x) < 1.e-6)			// zero test for floating point numbers
#define DYNA_ISEQUAL(x,y)	(fabs((x) - (y)) < 1.e-6) // test for equality of float numbers
#define DYNA_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
#define DYNA_RAND(l,u)	((double)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#define DYNA_RAND_INT(l,u)  (rand() % (u - l) + l)  // int random number in interval [l,u) including l, excluding u


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <ctime>

namespace dynacore {
  typedef Eigen::Transform<double, 3, Eigen::Affine> Transform;
  typedef Eigen::Translation3d Translation;
  typedef Eigen::Quaternion<double> Quaternion;
  typedef Eigen::Matrix<double,2,1> Vect2;
  typedef Eigen::Matrix<double,3,1> Vect3;
  typedef Eigen::Matrix<double,4,1> Vect4;
  typedef Eigen::VectorXd Vector;
  typedef Eigen::MatrixXd Matrix;
  // Euler angle (Yaw, Pitch, Roll) to Quaternion
  void convert(double yaw, double pitch, double roll, dynacore::Quaternion& to);
  // Quaternion to Euler ZYX
  void convert(const dynacore::Quaternion& from, double & yaw, double & pitch, double & roll);
  // Quaternion to so(3)
  void convert(dynacore::Quaternion const & from, dynacore::Vect3 & to);

  // so(3) to Quaternion
  void convert(dynacore::Vect3 const & from, dynacore::Quaternion & to);
  // dynacore::Vector to std::vector
  void convert(dynacore::Vector const & from, std::vector<double> & to);
  // std::vector to dynacore::Vector
  void convert(std::vector<double> const & from, dynacore::Vector & to);
  // double array to dynacore::Vector
  void convert(double const * from, size_t length, dynacore::Vector & to);

  // Note: when QuatMultiply is used to rotate a vector, set bound_pi to false.
  Quaternion QuatMultiply(const Quaternion & q1, const Quaternion & q2, bool bound_pi = true);

  bool compare(dynacore::Matrix const & lhs, dynacore::Matrix const & rhs, double precision);
  bool compare(dynacore::Quaternion const & lhs, dynacore::Quaternion const & rhs, double precision);

  double _bind_half_pi(double);

  void Copy(const dynacore::Vector & sub, double* obj);
  void Copy(const double* sub, double* obj, int dim);
  void SetArrayZero(double* array, int dim);
  double Dot(const double* a, const double * b, int dim);
  // Signum returns (-1, 0, or 1) depending on the sign of the argument
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0)) ;
  }
}

#endif

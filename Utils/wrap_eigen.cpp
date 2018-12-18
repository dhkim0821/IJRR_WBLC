/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file dynacore/wrap_eigen.cpp
   \author Roland Philippsen
*/
#include "wrap_eigen.hpp"
#include <cmath>
#include <iostream>
using namespace std;

namespace dynacore {
    void convert(dynacore::Quaternion const & from, dynacore::Vect3 & to){
        double w = from.w();
        Vect3 img_vec;
        img_vec[0] = from.x();
        img_vec[1] = from.y();
        img_vec[2] = from.z();
        // double theta = 2.0*acos(w);
        double theta = 2.0*asin(sqrt(img_vec[0] * img_vec[0] +
                                     img_vec[1] * img_vec[1] +
                                     img_vec[2] * img_vec[2]) );
        
        if(fabs(theta)<0.0000001) {
            to = dynacore::Vector::Zero(3);
            return ;
        }
        to = img_vec/sin(theta/2.0);
        // printf("quaternion (theta, w, x, y, z): %f, %f, %f, %f, %f\n", theta, w, to[0], to[1], to[2]);
        
        // if(theta > M_PI){
        //     theta -= 2*M_PI;
        // }
        // if(theta < -M_PI){
        //     theta += 2*M_PI;
        // }
        to = to * theta;
    }

    void convert(dynacore::Vect3 const & from, dynacore::Quaternion & to){
        double theta = from[0]*from[0] + from[1]*from[1] + from[2]*from[2];
        theta = sqrt(theta);

        if(theta > 1.0e-9){
            to.w() = cos(theta/2.0);
            to.x() = sin(theta/2.0) * from[0]/theta;
            to.y() = sin(theta/2.0) * from[1]/theta;
            to.z() = sin(theta/2.0) * from[2]/theta;
        }
        else{
            to.w() = 1.0;
            to.x() = 0.5 * from[0];
            to.y() = 0.5 * from[1];
            to.z() = 0.5 * from[2];
        }
    }

  // Note: when QuatMultiply is used to rotate a vector, set bound_pi to false.
  Quaternion QuatMultiply(const Quaternion & q1, const Quaternion & q2, bool bound_pi){
        Quaternion ret_q(
            q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z(),
            q1.w()*q2.x() + q1.x()*q2.w() + q1.y()*q2.z() - q1.z()*q2.y(),
            q1.w()*q2.y() - q1.x()*q2.z() + q1.y()*q2.w() + q1.z()*q2.x(),
            q1.w()*q2.z() + q1.x()*q2.y() - q1.y()*q2.x() + q1.z()*q2.w());
        
        if(ret_q.w() < 0 && bound_pi){
          ret_q.w() *= -1.;
          ret_q.x() *= -1.;
          ret_q.y() *= -1.;
          ret_q.z() *= -1.;
        }

        return ret_q;
    }

    // -pi/2 ~ pi/2
    void convert(const dynacore::Quaternion & from, double & yaw, double & pitch, double & roll){
        Quaternion Qfrom = from;
        Qfrom.normalize();
        Eigen::Matrix3d mat = from.toRotationMatrix();
        Vector ea =  mat.eulerAngles(2, 1, 0);
        yaw = ea[0];
        pitch = ea[1];
        roll = ea[2];

        // Vector ea =  mat.eulerAngles(0, 1, 2);
        // roll = ea[0];
        // pitch = ea[1];
        // yaw = ea[2];

        // yaw = _bind_half_pi(yaw);
        // pitch = _bind_half_pi(pitch);
        // roll = _bind_half_pi(roll);
    }

    double _bind_half_pi(double ang){
        if(ang > M_PI/2){
            return ang - M_PI;
        }
        if(ang < -M_PI/2){
            return ang + M_PI;
        }
        return ang;
    }
    void convert(double yaw, double pitch, double roll, dynacore::Quaternion& to){
        // double c1 = cos(yaw);
        // double s1 = sin(yaw);
        // double c2 = cos(pitch);
        // double s2 = sin(pitch);
        // double c3 = cos(roll);
        // double s3 = sin(roll);
        // double w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
        // double w4 = (4.0 * w);
        // double x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
        // double y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
        // double z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
        // to = dynacore::Quaternion(w, x, y, z);
        Quaternion Qyaw(cos(0.5*yaw), 0, 0, sin(0.5*yaw) );
        Quaternion Qpitch(cos(0.5*pitch), 0, sin(0.5*pitch), 0) ;
        Quaternion Qroll(cos(0.5*roll), sin(0.5*roll), 0, 0);

        to = Qyaw * Qpitch * Qroll;
    }
    bool compare(dynacore::Matrix const & lhs, dynacore::Matrix const & rhs, double precision)
    {
        if ( &lhs == &rhs ) {
            return true;
        }
        if ( lhs.rows() != rhs.rows() ) {
            return false;
        }
        if ( lhs.cols() != rhs.cols() ) {
            return false;
        }
        for (int ii(0); ii < lhs.rows(); ++ii) {
            for (int jj(0); jj < lhs.cols(); ++jj) {
                if (fabs(lhs.coeff(ii, jj) - rhs.coeff(ii, jj)) > precision) {
                    return false;
                }
            }
        }
        return true;
    }

    bool compare(dynacore::Quaternion const & lhs, dynacore::Quaternion const & rhs, double precision)
    {
        return compare(lhs.coeffs(), rhs.coeffs(), precision);
    }

    void convert(dynacore::Vector const & from, std::vector<double> & to)
    {
        to.resize(from.size());
        Vector::Map(&to[0], to.size()) = from;
    }

    void convert(std::vector<double> const & from, dynacore::Vector & to)
    {
        to = Vector::Map(&from[0], from.size());
    }

    void convert(double const * from, size_t length, dynacore::Vector & to)
    {
        to = Vector::Map(from, length);
    }

  void Copy(const dynacore::Vector & sub, double* obj){
    for(int i(0);i<sub.rows(); ++i){    obj[i] = sub[i];    }
  }
  void Copy(const double* sub, double* obj, int dim){
    for(int i(0); i<dim; ++i){ obj[i] = sub[i]; }
  }
  void SetArrayZero(double* array, int dim){
    for(int i(0); i<dim; ++i){ array[i] = 0. ; }
  }
  double Dot(const double* a, const double * b, int dim){
    double sum(0.);
    for(int i(0); i<dim; ++i){ sum += a[i] * b[i]; }
    return sum;
  }
}

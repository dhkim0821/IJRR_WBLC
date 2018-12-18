#include "LQR.hpp"
#include <Eigen/Eigenvalues>
#include <iostream>

using namespace std;

namespace dynacore{
    void LQR(const Matrix & A,
             const Matrix & B,
             const Matrix & Q,
             const Matrix & R,
             Matrix & S, bool descrete){
        int n = A.rows();
        Matrix Z(2*n, 2*n);

        if(!descrete){
            Z.block(0, 0, n, n) = A;

            Z.block(0, n, n, n) = -B * R.inverse() * B.transpose();

            Z.block(n, 0, n, n) = -Q;

            Z.block(n, n, n, n) = -A.transpose();

        } else {

            Matrix Ainv = A.inverse();
            Z.block(0, 0, n, n) = A + B * R.inverse() * B.transpose() * Ainv.transpose() * Q;
            Z.block(0, n, n, n) = -B * R.inverse() * B.transpose() * Ainv.transpose();
            Z.block(n, 0, n, n) = -Ainv.transpose() * Q;
            Z.block(n, n, n, n) = Ainv.transpose();

        }

        Eigen::EigenSolver<Matrix> es(Z);
        Eigen::MatrixXcd V(2*n, n);

        int col_num(0);
        for (int i(0); i<2*n; ++i){
            if(!descrete){
                if( (es.eigenvalues()[i]).real() < 0.0){
                    V.col(col_num) = es.eigenvectors().col(i);
                    ++col_num;
                }
            } else {
                if( ((es.eigenvalues()[i]).real())*( (es.eigenvalues()[i]).real()) < 1.0){
                    V.col(col_num) = es.eigenvectors().col(i);
                    ++col_num;
                }
            }
        }
        // cout << "The eigenvalues of Z are:" << endl << es.eigenvalues() << endl;
        // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
        // cout << "Finally, S = " << endl << S << endl;

        Eigen::MatrixXcd Stmp = V.block(n, 0, n, n) * (V.block(0, 0, n, n)).inverse();
        S = Stmp.real();
    }
}

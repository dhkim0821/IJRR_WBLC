#ifndef MINJERK_ONE_DIM
#define MINJERK_ONE_DIM

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>

// Implements the polynomial:
// x(t) = a0 + a_1*t + a_2*t^2 + a_3*t^3 + a_4*t^4 + a_5*t^5
// Needs boundary conditions x(to), xdot(to), xddot(to), x(tf), xdot(tf), xddot(tf)
// 		where to and tf are the initial and final times respectively. Also, tf > to.

class MinJerk_OneDimension{
public:
	// Constructors
	MinJerk_OneDimension();
	MinJerk_OneDimension(const dynacore::Vect3 & init, const dynacore::Vect3 & end, 
						 const double & time_start, const double & time_end);

	void setParams(const dynacore::Vect3 & init, const dynacore::Vect3 & end, 
 			  const double & time_start, const double & time_end);

	// Functions
	void test_func();

	void getPos(const double & time, double & pos);
	void getVel(const double & time, double & vel);	
	void getAcc(const double & time, double & acc);	

	void printParameters();

	// Destructor
	~MinJerk_OneDimension();


private:
	dynacore::Matrix C_mat; // Matrix of Coefficients
	dynacore::Matrix C_mat_inv;  // Inverse of Matrix of Coefficients
	dynacore::Vector a_coeffs;   // mininum jerk coeffs. a = [a0, a1, a2, a3, a4, a5, a6];
	dynacore::Vector bound_cond; // boundary conditions x_b = [ x(to), xdot(to), xddot(to), x(tf), xdot(tf), xddot(tf)]

	dynacore::Vect3 init_cond;  // initial pos, vel, acceleration
	dynacore::Vect3 end_cond;   // final pos, vel, acceleration
	double to; // Starting time
	double tf; // Ending time

	void Initialization();

	// Compute the coefficients
	void compute_coeffs();


};

#endif
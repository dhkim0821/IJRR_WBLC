
#include "srDyn/srConstraint.h"

#include "srDyn/srSystem.h"
#include "LieGroup/_array.h"


//-- LCP Solver PGS
#define LCP_PGS_RANDOMLY_REORDER_CONSTRAINTS		1
#define LCP_PGS_OPTION_DEFAULT_ITERMAX				30
#define LCP_PGS_OPTION_DEFAULT_SOR_W				0.9
#define LCP_PGS_OPTION_DEFAULT_EPS_EA				1E-3
#define LCP_PGS_OPTION_DEFAULT_EPS_RESIDUAL			1E-6
#define LCP_PGS_OPTION_DEFAULT_EPS_DIVIDE			1E-9
//***********************************************//

struct PGSOption
{
	int itermax;
	sr_real sor_w;
	sr_real eps_ea;
	sr_real eps_res;
	sr_real eps_div;

	void SetDefault();
};

bool	LCPSolver_PGS____MARK3(int n, int nskip, int nub, sr_real * A, sr_real * x, sr_real * b, 
							   sr_real * lo, sr_real * hi, int * findex, PGSOption * option);
bool	LCPSolver_PGS____MARK4(int n, int nskip, int nub, sr_real * A, sr_real * x, sr_real * b, 
							   sr_real * lo, sr_real * hi, int * findex, PGSOption * option);

bool	LCPSolver_SPARSE_PGS____MARK1(int n, int * r, int * c, int * rowofs, sr_real * A,
									  sr_real * x, sr_real * b, sr_real * lo, sr_real * hi,
									  int * findex, PGSOption * option);

//	given (A,b,lo,hi), solve the LCP problem: A*x = b+w, where each x(i),w(i)
//	satisfies one of
//		(1) x = lo, w >= 0
//		(2) x = hi, w <= 0
//		(3) lo < x < hi, w = 0
//	A is a matrix of dimension n*n, everything else is a vector of size n*1.
//	lo and hi can be +/- dInfinity as needed. the first `nub' variables are
//	unbounded, i.e. hi and lo are assumed to be +/- dInfinity.
//
//	we restrict lo(i) <= 0 and hi(i) >= 0.
//
//	the original data (A,b) may be modified by this function.
//
//	if the `findex' (friction index) parameter is non-negative, it points to an array
//	of index values. in this case constraints that have findex[i] >= 0, the lo and hi values
//	for the special constraints are set:
//	hi[i] = abs( hi[i] * x[findex[i]] )
//	lo[i] = -hi[i]
//	and the solution continues. this mechanism allows a friction approximation
//	to be implemented. the first `nub' variables are assumed to have findex < 0.



class ISLAND
{
public:
	// MARK8
	ConstraintInfo					m_cinfo;
	srSystem*						m_ID;

	ConstraintPtrArray				m_AllConstraints;
	int								m_nConstraints;

	void	_PGS_Solve_Constraint_____________________MARK7(PGSOption * option);

	void	_SPARSE_PGS_Solve_Constraint______________MARK11(PGSOption * option);
};

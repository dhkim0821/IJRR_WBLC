#include "srDyn/srISLAND.h"


//***********************************************//
// Operations
#define ALLOC_BK(type,variable,size) type * variable = new type[size]
#define FREE_BK(variable) delete[] variable
#define dPAD_BK(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))
#define dSetZero_BK(a,n,n_dummy) n_dummy = n;while (--n_dummy >= 0) {a[n_dummy] = 0;}
#define dSetValue_BK(a,n,n_dummy,val) n_dummy = n;while (--n_dummy >= 0) {a[n_dummy] = val;}
//***********************************************//



//**********************************************************************//
// Random Number Generator
static unsigned long seed = 0;
unsigned long dRand()
{
	seed = (1664525L*seed + 1013904223L) & 0xffffffff;
	return seed;
}

// adam's all-int straightforward(?) dRandInt (0..n-1)
int dRandInt (int n)
{
	// seems good; xor-fold and modulus
	const unsigned long un = n;
	unsigned long r = dRand();

	// note: probably more aggressive than it needs to be -- might be
	//       able to get away without one or two of the innermost branches.
	if (un <= 0x00010000UL) {
		r ^= (r >> 16);
		if (un <= 0x00000100UL) {
			r ^= (r >> 8);
			if (un <= 0x00000010UL) {
				r ^= (r >> 4);
				if (un <= 0x00000004UL) {
					r ^= (r >> 2);
					if (un <= 0x00000002UL) {
						r ^= (r >> 1);
					}
				}
			}
		}
	}

	return (int) (r % un);    
}
//**********************************************************************//



//**********************************************************************//
// LCP Solvers
void	PGSOption::SetDefault()
{
	itermax = LCP_PGS_OPTION_DEFAULT_ITERMAX;
	sor_w = LCP_PGS_OPTION_DEFAULT_SOR_W;
	eps_ea = LCP_PGS_OPTION_DEFAULT_EPS_EA;
	eps_res = LCP_PGS_OPTION_DEFAULT_EPS_RESIDUAL;
	eps_div = LCP_PGS_OPTION_DEFAULT_EPS_DIVIDE;
}

bool	LCPSolver_SPARSE_PGS____MARK1(int n, int * r, int * c, int * rowofs, sr_real * A,
									  sr_real * x, sr_real * b, sr_real * lo, sr_real * hi,
									  int * findex, PGSOption * option)
{
	// ±¸ÇöÁß
	return false;
}

bool	LCPSolver_PGS____MARK3(int n, int nskip, int nub, sr_real * A, sr_real * x, sr_real * b, 
							   sr_real * lo, sr_real * hi, int * findex, PGSOption * option)
{
	// LDLT solver will work !!!
	//if (nub == n)
	//{
	//	return LDLTSolver(n,nskip,A,x,b)
	//}

	int i, j, iter, idx;
	bool sentinel;
	sr_real old_x, new_x, hi_tmp, lo_tmp, dummy, ea;
	sr_real * A_ptr;
	sr_real one_minus_sor_w = 1.0 - (option->sor_w);

	//--- Initial Loop & Test
	sentinel = true;
	//- unbounded constraint
	for (i = 0 ; i < nub ; i++)
	{
		A_ptr = A + nskip*i;
		new_x = b[i];
		old_x = x[i];

		for (j = 0 ; j < i ; j++)
			new_x -= A_ptr[j]*x[j];
		for (j = i + 1 ; j < n ; j++)
			new_x -= A_ptr[j]*x[j];

		x[i] = new_x/A[nskip*i + i];

		if (sentinel)
		{
			ea = fabs(x[i] - old_x);
			if (ea > option->eps_res)
				sentinel = false;
		}
	}
	//- bounded constraint
	for (i = nub ; i < n ; i++)
	{
		A_ptr = A + nskip*i;
		new_x = b[i];
		old_x = x[i];

		for (j = 0 ; j < i ; j++)
			new_x -= A_ptr[j]*x[j];
		for (j = i + 1 ; j < n ; j++)
			new_x -= A_ptr[j]*x[j];

		new_x = new_x/A[nskip*i + i];

		if (findex[i] >= 0)	// friction index
		{
			hi_tmp = hi[i] * x[findex[i]];
			lo_tmp = -hi_tmp;

			if (new_x > hi_tmp)
				x[i] = hi_tmp;
			else if (new_x < lo_tmp)
				x[i] = lo_tmp;
			else
				x[i] = new_x;
		}
		else					// no friction index
		{
			if (new_x > hi[i])
				x[i] = hi[i];
			else if (new_x < lo[i])
				x[i] = lo[i];
			else
				x[i] = new_x;
		}

		if (sentinel)
		{
			ea = fabs(x[i] - old_x);
			if (ea > option->eps_res)
				sentinel = false;
		}
	}
	if (sentinel)
		return true;

	//--- SORTING & SCALING
	ALLOC_BK(int,order,n);
	for (i = 0 ; i < n ; i++)
	{
		order[i] = i;

		dummy = 1.0/A[nskip*i + i];  // diagonal element
		b[i] *= dummy;
		for (j = 0 ; j < n ; j++)
		{
			A[nskip*i + j] *= dummy;
		}
	}

	//--- ITERATION LOOP
	for (iter = 1 ; iter < option->itermax ; iter++)
	{
		//--- RANDOMLY_REORDER_CONSTRAINTS
#if LCP_PGS_RANDOMLY_REORDER_CONSTRAINTS
		if ((iter & 7)==0) 
		{
			for (i=1; i<n; ++i) 
			{
				int tmp = order[i];
				int swapi = dRandInt(i+1);
				order[i] = order[swapi];
				order[swapi] = tmp;
			}
		}
#endif

		sentinel = true;

		//-- ONE LOOP
		for (i = 0 ; i < n ; i++)
		{
			idx = order[i];

			A_ptr = A + nskip*idx;
			new_x = b[idx];
			old_x = x[idx];

			for (j = 0 ; j < idx ; j++)
				new_x -= A_ptr[j]*x[j];
			for (j = idx + 1 ; j < n ; j++)
				new_x -= A_ptr[j]*x[j];

			new_x = (option->sor_w * new_x) + (one_minus_sor_w * old_x);

			if (findex[idx] >= 0)	// friction index
			{
				hi_tmp = hi[idx] * x[findex[idx]];
				lo_tmp = -hi_tmp;

				if (new_x > hi_tmp)
					x[idx] = hi_tmp;
				else if (new_x < lo_tmp)
					x[idx] = lo_tmp;
				else
					x[idx] = new_x;
			}
			else					// no friction index
			{
				if (new_x > hi[idx])
					x[idx] = hi[idx];
				else if (new_x < lo[idx])
					x[idx] = lo[idx];
				else
					x[idx] = new_x;
			}

			if ( sentinel && fabs(x[idx]) > option->eps_div )
			{
				ea = fabs((x[idx] - old_x)/x[idx]);
				if (ea > option->eps_ea)
					sentinel = false;
			}
		}

		if (sentinel)
			break;
	}
	FREE_BK(order);
	return sentinel;
}

bool	LCPSolver_PGS____MARK4(int n, int nskip, int /*nub*/, sr_real * A, sr_real * x, sr_real * b, 
							   sr_real * lo, sr_real * hi, int * findex, PGSOption * option)
{
	// LDLT solver will work !!!
	//if (nub == n)
	//{
	//	return LDLTSolver(n,nskip,A,x,b)
	//}

	int i, j, iter, idx, n_new;
	bool sentinel;
	sr_real old_x, new_x, hi_tmp, lo_tmp, dummy, ea;
	sr_real * A_ptr;
	sr_real one_minus_sor_w = 1.0 - (option->sor_w);


	//--- ORDERING & SCALING & INITIAL LOOP & Test
	ALLOC_BK(int,order,n);

	n_new = 0;
	sentinel = true;
	for (i = 0 ; i < n ; i++)
	{
		// ORDERING
		if ( A[nskip*i + i] < option->eps_div )
		{
			x[i] = 0.0;
			continue;
		}
		order[n_new++] = i;

		// INITIAL LOOP
		A_ptr = A + nskip*i;
		new_x = b[i];
		old_x = x[i];

		for (j = 0 ; j < i ; j++)
			new_x -= A_ptr[j]*x[j];
		for (j = i + 1 ; j < n ; j++)
			new_x -= A_ptr[j]*x[j];

		new_x = new_x/A[nskip*i + i];

		if (findex[i] >= 0)	// friction index
		{
			hi_tmp = hi[i] * x[findex[i]];
			lo_tmp = -hi_tmp;

			if (new_x > hi_tmp)
				x[i] = hi_tmp;
			else if (new_x < lo_tmp)
				x[i] = lo_tmp;
			else
				x[i] = new_x;
		}
		else					// no friction index
		{
			if (new_x > hi[i])
				x[i] = hi[i];
			else if (new_x < lo[i])
				x[i] = lo[i];
			else
				x[i] = new_x;
		}

		// TEST
		if (sentinel)
		{
			ea = fabs(x[i] - old_x);
			if (ea > option->eps_res)
				sentinel = false;
		}
	}
	if (sentinel)
	{
		FREE_BK(order);
		return true;
	}

	// SCALING
	for (i = 0 ; i < n_new ; i++)
	{
		idx = order[i];

		dummy = 1.0/A[nskip*idx + idx];  // diagonal element
		b[idx] *= dummy;
		for (j = 0 ; j < n ; j++)
			A[nskip*idx + j] *= dummy;
	}

	//--- ITERATION LOOP
	for (iter = 1 ; iter < option->itermax ; iter++)
	{
		//--- RANDOMLY_REORDER_CONSTRAINTS
#if LCP_PGS_RANDOMLY_REORDER_CONSTRAINTS
		if ((iter & 7)==0) 
		{
			int tmp, swapi;
			for (i = 1 ; i < n_new ; i++) 
			{
				tmp = order[i];
				swapi = dRandInt(i+1);
				order[i] = order[swapi];
				order[swapi] = tmp;
			}
		}
#endif

		sentinel = true;

		//-- ONE LOOP
		for (i = 0 ; i < n_new ; i++)
		{
			idx = order[i];

			A_ptr = A + nskip*idx;
			new_x = b[idx];
			old_x = x[idx];

			for (j = 0 ; j < idx ; j++)
				new_x -= A_ptr[j]*x[j];
			for (j = idx + 1 ; j < n ; j++)
				new_x -= A_ptr[j]*x[j];

			new_x = (option->sor_w * new_x) + (one_minus_sor_w * old_x);

			if (findex[idx] >= 0)	// friction index
			{
				hi_tmp = hi[idx] * x[findex[idx]];
				lo_tmp = -hi_tmp;

				if (new_x > hi_tmp)
					x[idx] = hi_tmp;
				else if (new_x < lo_tmp)
					x[idx] = lo_tmp;
				else
					x[idx] = new_x;
			}
			else					// no friction index
			{
				if (new_x > hi[idx])
					x[idx] = hi[idx];
				else if (new_x < lo[idx])
					x[idx] = lo[idx];
				else
					x[idx] = new_x;
			}

			if ( sentinel && fabs(x[idx]) > option->eps_div)
			{
				ea = fabs((x[idx] - old_x)/x[idx]);
				if (ea > option->eps_ea)
					sentinel = false;
			}
		}

		if (sentinel)
			break;
	}
	FREE_BK(order);
	return sentinel;
}
//**********************************************************************//

//**********************************************************************//
// ISLAND
void ISLAND::_PGS_Solve_Constraint_____________________MARK7(PGSOption * option)
{
	if (m_nConstraints == 0)
	{
		return;
	}

	int i, j, k, m, n, nt, ntskip, nub;
	Constraint * pConstraint;

	ALLOC_BK(int, ofs, m_nConstraints);		// ofs : constraint dimension offset for constraint

	nt = nub = 0;
	for (i = 0 ; i < m_nConstraints ; i++)
	{
		pConstraint = m_AllConstraints[i];

		ofs[i] = nt;
		nt += pConstraint->nd;

		if (pConstraint->type1)
			nub += pConstraint->nd;
	}
	ntskip = dPAD_BK(nt);
	j = nt*ntskip;

	// lo, hi, findex, rhs, lambda, invMc
	ALLOC_BK(sr_real, lo, nt);			// lo : lower bound of lambda
	ALLOC_BK(sr_real, hi, nt);			// hi : upper bound of lambda
	ALLOC_BK(int, findex, nt);			// findex : friction index
	ALLOC_BK(sr_real, rhs,nt);			// rhs : c - J(v_old + invM*f_ext*h) where c is max( penet/h , -e*J*v_old )
	ALLOC_BK(sr_real, lambda, nt);		// lambda : constraint force
	ALLOC_BK(sr_real, invMc, j);			// ivMc : Constraint inverse mass matrix ( J*invM*Jt )

	dSetValue_BK(findex,nt,i,-1);
	//dSetZero_BK(invMc,j,i);
	
	for (i = 0 ; i < m_nConstraints ; i++)
	{
		pConstraint = m_AllConstraints[i];
		
		m_cinfo._lo = lo + ofs[i];
		m_cinfo._hi = hi + ofs[i];
		m_cinfo._findex = findex + ofs[i];
		m_cinfo._rhs = rhs + ofs[i];
		m_cinfo._lambda = lambda + ofs[i];

		// Fill 'lo, hi, findex, rhs, lambda'
		pConstraint->GetInformation(&m_cinfo);


		// Fill 'invMc'
		pConstraint->Excite();
		for (j = 0 ; j < pConstraint->nd ; j++) // Constraint Dimension Loop 
		{
			// Adjust findex for global index
			if (findex[ofs[i]+j] >=0)
				findex[ofs[i]+j] += ofs[i];

			pConstraint->ApplyImpulse(j);	// Apply impulse for impulse test


			for (k = i ; k < m_nConstraints ; k++)	// Internal Constraint Loop
			{
				m_AllConstraints[k]->GetDelVelocity(invMc
					+ (ntskip * (ofs[i]+j))	// row skip
					+ ofs[k] );				// column skip
			}

			// symmetric part
			for (k = 0 ; k < i ; k++)
			{
				n = m_AllConstraints[k]->nd;
				for (m = 0 ; m < n ; m++)
				{
					invMc[(ntskip * (ofs[i]+j)) + (ofs[k]+m)] = invMc[(ntskip * (ofs[k]+m)) + (ofs[i]+j)];
				}
			}
		}
		pConstraint->UnExcite();
	}

	LCPSolver_PGS____MARK4(nt,ntskip,nub,invMc,lambda,rhs,lo,hi,findex,option);


	for (i = 0 ; i < m_nConstraints ; i++)
	{
		pConstraint = m_AllConstraints[i];
		pConstraint->SetImpulse(lambda + ofs[i]);
		pConstraint->Excite();
	}

	FREE_BK(ofs);
	FREE_BK(lo);
	FREE_BK(hi);
	FREE_BK(findex);
	FREE_BK(rhs);
	FREE_BK(lambda);
	FREE_BK(invMc);
}




//**********************************************************************//

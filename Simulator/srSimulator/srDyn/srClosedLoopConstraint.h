#pragma once

#include "srDyn/srConstraint.h"


//**********************************************************************//
// Closed Loop
class ClosedLoop : public Constraint
{
public:
	ClosedLoop()
	{
		nd = 6;
		type1 = true;

		lambda[0] = 0.0;
		lambda[1] = 0.0;
		lambda[2] = 0.0;
		lambda[3] = 0.0;
		lambda[4] = 0.0;
		lambda[5] = 0.0;
	};

	// static members
	static void SetErp(sr_real _erp);
	static sr_real erp_closedloop;

	static void SetAllowedPenetration(sr_real _allowedpenetration);
	static sr_real allowederror;

	static void SetMaximumErpVelocity(sr_real _maximum_erp_velocity);
	static sr_real maximum_erp_velocity;

	// member variables
	static dse3 UnitImp[6]; // constraint jacobian : this is same for all closed loop, hence i chose to use static variable.
	sr_real	closedloopError[6];
	sr_real	lambda[6];

	srSystem	* pSystem;
	srLink	* pLeftMass;
	srLink	* pRightMass;

	SE3		HomeRelativeFrame;
	SE3		RelativeFrame;

	// member functions
	void		GetError(sr_real _recip_timestep);

	// virtual functions
	void		GetInformation(ConstraintInfo * info);
	void		ApplyImpulse(int _idx);
	void		GetDelVelocity(sr_real * sjari);
	void		Excite();
	void		UnExcite();
	void		SetImpulse(sr_real * _lambda);
	srSystem*	UF_Find_Constraint();

};
//**********************************************************************//


#include <memory.h>
#include "srDyn/srLink.h"
#include "srDyn/srSystem.h"
#include "srDyn/srClosedLoopConstraint.h"


//**********************************************************************//
// Closed Loop
dse3 ClosedLoop::UnitImp[6] = {	dse3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
								dse3(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
								dse3(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
								dse3(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
								dse3(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
								dse3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)	};

sr_real ClosedLoop::erp_closedloop = 0.8;
sr_real ClosedLoop::allowederror = 0.01;
sr_real ClosedLoop::maximum_erp_velocity = 10;

void ClosedLoop::SetErp(sr_real _erp)
{
	erp_closedloop = _erp;
}

void ClosedLoop::SetAllowedPenetration(sr_real _allowedpenetration)
{
	allowederror = _allowedpenetration;
}

void ClosedLoop::SetMaximumErpVelocity(sr_real _maximum_erp_velocity)
{
	maximum_erp_velocity = _maximum_erp_velocity;
}

inline void ClosedLoop::GetError(sr_real _recip_timestep)
{
	se3 tmp_err = Log(HomeRelativeFrame % RelativeFrame);
	tmp_err = _recip_timestep * erp_closedloop * Ad(RelativeFrame,tmp_err);

	closedloopError[0] = tmp_err[0];
	closedloopError[1] = tmp_err[1];
	closedloopError[2] = tmp_err[2];
	closedloopError[3] = tmp_err[3];
	closedloopError[4] = tmp_err[4];
	closedloopError[5] = tmp_err[5];
}

void ClosedLoop::GetInformation(ConstraintInfo * info)
{

	//-- lo, hi, findex
	info->_lo[0] = NINFINITY_BK;
	info->_lo[1] = NINFINITY_BK;
	info->_lo[2] = NINFINITY_BK;
	info->_lo[3] = NINFINITY_BK;
	info->_lo[4] = NINFINITY_BK;
	info->_lo[5] = NINFINITY_BK;

	info->_hi[0] = PINFINITY_BK;
	info->_hi[1] = PINFINITY_BK;
	info->_hi[2] = PINFINITY_BK;
	info->_hi[3] = PINFINITY_BK;
	info->_hi[4] = PINFINITY_BK;
	info->_hi[5] = PINFINITY_BK;

	//-- rhs

	// error reduction
	GetError(info->recip_timestep);

	// relative velocity
	se3 tmp_negrelvel = Ad(RelativeFrame,pRightMass->m_Vel) - pLeftMass->m_Vel;
	
	info->_rhs[0] = tmp_negrelvel[0] + closedloopError[0];
	info->_rhs[1] = tmp_negrelvel[1] + closedloopError[1];
	info->_rhs[2] = tmp_negrelvel[2] + closedloopError[2];
	info->_rhs[3] = tmp_negrelvel[3] + closedloopError[3];
	info->_rhs[4] = tmp_negrelvel[4] + closedloopError[4];
	info->_rhs[5] = tmp_negrelvel[5] + closedloopError[5];

	//-- lambda

	memcpy(info->_lambda,lambda,6*sizeof(sr_real));
}

void ClosedLoop::ApplyImpulse(int _idx)
{
	pSystem->FS_Reset_Bias_T();
	pSystem->FS_UpdateBiasImpulse(pLeftMass,UnitImp[_idx]);
	pSystem->FS_UpdateBiasImpulse(pRightMass,dAd(RelativeFrame , -UnitImp[_idx]));
	pSystem->FS_UpdateDelVelocity();
}

void ClosedLoop::GetDelVelocity(sr_real * sjari)
{
	if (pSystem->m_bExcited)
	{
		// Need operator like (sr_real* = (se3&))
		se3 tmp = pLeftMass->m_DelVel - Ad(RelativeFrame,pRightMass->m_DelVel);

		sjari[0] = tmp[0];
		sjari[1] = tmp[1];
		sjari[2] = tmp[2];
		sjari[3] = tmp[3];
		sjari[4] = tmp[4];
		sjari[5] = tmp[5];
	}
	else
	{
		sjari[0] = 0.0;
		sjari[1] = 0.0;
		sjari[2] = 0.0;
		sjari[3] = 0.0;
		sjari[4] = 0.0;
		sjari[5] = 0.0;
	}
}

void ClosedLoop::Excite()
{
	pSystem->ExciteSystem();
}

void ClosedLoop::UnExcite()
{
	pSystem->UnExciteSystem();
}

void ClosedLoop::SetImpulse(sr_real * _lambda)
{
	memcpy(lambda,_lambda,6*sizeof(sr_real));

	dse3 tmp(_lambda);
	pLeftMass->m_ConstraintImpulse += tmp;
	pRightMass->m_ConstraintImpulse -= dAd(RelativeFrame, tmp);
}

srSystem* ClosedLoop::UF_Find_Constraint()
{
	return pSystem->m_UF_id;
}
//**********************************************************************//


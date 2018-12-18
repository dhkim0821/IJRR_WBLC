#include "srDyn/srJointConstraint.h"


//**********************************************************************//
// Joint Position Limit Constraint

sr_real JointConstraint::erp_jointpositionlimit = 0.8;
sr_real JointConstraint::allowedjointerror = 0.0001;
sr_real JointConstraint::bouncing_threshold = 0.2;
sr_real JointConstraint::maximum_erp_velocity = 10;
sr_real JointConstraint::maximum_bouncing_velocity = 20;

void JointConstraint::SetErp(sr_real _erp)
{
	erp_jointpositionlimit = _erp;
}

void JointConstraint::SetAllowedPenetration(sr_real _allowedpenetration)
{
	allowedjointerror = _allowedpenetration;
}

void JointConstraint::SetBouncingThreshold(sr_real _bouncingthreshold)
{
	bouncing_threshold = _bouncingthreshold;
}

void JointConstraint::SetMaximumErpVelocity(sr_real _maximum_erp_velocity)
{
	maximum_erp_velocity = _maximum_erp_velocity;
}

void JointConstraint::SetMaximumBouncingVelocity(sr_real _maximum_bouncing_velocity)
{
	maximum_bouncing_velocity = _maximum_bouncing_velocity;
}

//////////////////////////////////////////////////////////////////////////
// Virtual Functions
bool JointConstraint::_inspect_R_PositionLimit()
{
	// Lower Limit check
	LimitError = Limit[0] - m_pRstate->m_rValue[0];
	if (LimitError >= 0)
	{
		Negative_Velocity = -m_pRstate->m_rValue[1];

		ForceLimit[0] = 0.0;
		ForceLimit[1] = PINFINITY_BK;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	// Upper Limit check
	LimitError = Limit[1] - m_pRstate->m_rValue[0];
	if (LimitError <= 0)
	{
		Negative_Velocity = -m_pRstate->m_rValue[1];

		ForceLimit[0] = NINFINITY_BK;
		ForceLimit[1] = 0.0;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	bActive = false;
	return bActive;
}

bool JointConstraint::_inspect_R_TorqueLimit()
{
	if ( pJoint->m_IsDeviceOn )
	{
		Negative_Velocity = -m_pRstate->m_rValue[1] + m_pRstate->m_rCommand;
		
		//ForceLimit[0] = m_pRstate->m_TorqueLimit[0];
		//ForceLimit[1] = m_pRstate->m_TorqueLimit[1];

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	bActive = false;
	return bActive;
}

bool JointConstraint::_inspect_P_PositionLimit()
{
	// Lower Limit check
	LimitError = Limit[0] - m_pPstate->m_rValue[0];
	if (LimitError >= 0 )
	{
		Negative_Velocity = -m_pPstate->m_rValue[1];

		ForceLimit[0] = 0.0;
		ForceLimit[1] = PINFINITY_BK;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}

	// Upper Limit check
	LimitError = Limit[1] - m_pPstate->m_rValue[0];
	if (LimitError <= 0)
	{
		Negative_Velocity = -m_pPstate->m_rValue[1];

		ForceLimit[0] = NINFINITY_BK;
		ForceLimit[1] = 0.0;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}
	bActive = false;


	return bActive;
}

bool JointConstraint::_inspect_P_TorqueLimit()
{
	if ( pJoint->m_IsDeviceOn )
	{
		Negative_Velocity = -m_pPstate->m_rValue[1] + m_pPstate->m_rCommand;

		//ForceLimit[0] = m_pRstate->m_TorqueLimit[0];
		//ForceLimit[1] = m_pRstate->m_TorqueLimit[1];

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	bActive = false;
	return bActive;
}

bool JointConstraint::_inspect_U1_PositionLimit()
{
	LimitError = Limit[0] - m_pUstate->m_rValue[0][0];
	if (LimitError >= 0)
	{
		Negative_Velocity = -m_pUstate->m_rValue[0][1];

		ForceLimit[0] = 0.0;
		ForceLimit[1] = PINFINITY_BK;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}
	
	LimitError = Limit[1] - m_pUstate->m_rValue[0][0];
	if (LimitError <= 0)
	{
		Negative_Velocity = -m_pUstate->m_rValue[0][1];

		ForceLimit[0] = NINFINITY_BK;
		ForceLimit[1] = 0.0;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}
	bActive = false;

	return bActive;
}

bool JointConstraint::_inspect_U1_TorqueLimit()
{
	if ( pJoint->m_IsDeviceOn )
	{
		Negative_Velocity = -m_pUstate->m_rValue[0][1] + m_pUstate->m_rCommand[0];

		//ForceLimit[0] = m_pRstate->m_TorqueLimit[0];
		//ForceLimit[1] = m_pRstate->m_TorqueLimit[1];

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	bActive = false;
	return bActive;
}

bool JointConstraint::_inspect_U2_PositionLimit()
{
	LimitError = Limit[0] - m_pUstate->m_rValue[1][0];
	if (LimitError >= 0)
	{
		Negative_Velocity = -m_pUstate->m_rValue[1][1];

		ForceLimit[0] = 0.0;
		ForceLimit[1] = PINFINITY_BK;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}
	
	LimitError = Limit[1] - m_pUstate->m_rValue[1][0];
	if (LimitError <= 0)
	{
		Negative_Velocity = -m_pUstate->m_rValue[1][1];

		ForceLimit[0] = NINFINITY_BK;
		ForceLimit[1] = 0.0;

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}
	}
	bActive = false;
	
	return bActive;
}

bool JointConstraint::_inspect_U2_TorqueLimit()
{
	if ( pJoint->m_IsDeviceOn )
	{
		Negative_Velocity = -m_pUstate->m_rValue[1][1] + m_pUstate->m_rCommand[1];

		//ForceLimit[0] = m_pRstate->m_TorqueLimit[0];
		//ForceLimit[1] = m_pRstate->m_TorqueLimit[1];

		if (bActive)
		{
			lifetime++;
		}
		else // bActive == false
		{
			bActive = true;
			lifetime = 0;
		}

		return bActive;
	}

	bActive = false;
	return bActive;
}

bool JointConstraint::_inspect_B_Yaw()
{
	// 미구현
	return false;
}

bool JointConstraint::_inspect_B_RollPitch()
{
	// 미구현
	return false;
}

bool JointConstraint::Inspect_JointState()
{
	return (this->*m_pfn_inspect_jointstate)();
}

void JointConstraint::_getInformation_PositionLimit(ConstraintInfo * info)
{
	//-- lo, hi, findex
	info->_lo[0] = ForceLimit[0];
	info->_hi[0] = ForceLimit[1];

	//-- rhsx
	bool sign = true; // plus
	sr_real c = LimitError;
	if (LimitError < 0)
	{
		sign = false; // minus
		c *= -1;
	}

	c -= allowedjointerror;

	if (c < 0.0)
	{
		c = 0.0;
	}
	else
	{
		c *= info->recip_timestep * erp_jointpositionlimit;
		if (c > maximum_erp_velocity)
		{
			c = maximum_erp_velocity;
		}

		if (!sign)
		{
			c *= -1;
		}
	}

	info->_rhs[0] = Negative_Velocity + c;

	//-- lambda
	if (lifetime)
		info->_lambda[0] = lambda;
	else
		info->_lambda[0] = 0.0;
}

void JointConstraint::_getInformation_TorqueLimit(ConstraintInfo * info)
{
	//-- lo, hi, findex
	info->_lo[0] = ForceLimit[0];
	info->_hi[0] = ForceLimit[1];


	info->_rhs[0] = Negative_Velocity;

	//-- lambda
	if (lifetime)
		info->_lambda[0] = lambda;
	else
		info->_lambda[0] = 0.0;
}

void JointConstraint::GetInformation(ConstraintInfo * info)
{
	(this->*m_pfn_getInformation)(info);
}


void JointConstraint::_applyimpulse_R(int /* _idx */)
{
	m_pRstate->m_rImp = 1.0;

	pSystem->FS_Reset_Bias_T();
	pSystem->FS_UpdateBiasImpulse(pJoint->m_ChildLink);
	pSystem->FS_UpdateDelVelocity();
}

void JointConstraint::_applyimpulse_P(int /* _idx */)
{
	m_pPstate->m_rImp = 1.0;

	pSystem->FS_Reset_Bias_T();
	pSystem->FS_UpdateBiasImpulse(pJoint->m_ChildLink);
	pSystem->FS_UpdateDelVelocity();
}

void JointConstraint::_applyimpulse_U_1(int /* _idx */)
{
	m_pUstate->m_rImp[0] = 1.0;

	pSystem->FS_Reset_Bias_T();
	pSystem->FS_UpdateBiasImpulse(pJoint->m_ChildLink);
	pSystem->FS_UpdateDelVelocity();
}

void JointConstraint::_applyimpulse_U_2(int /* _idx */)
{
	m_pUstate->m_rImp[1] = 1.0;

	pSystem->FS_Reset_Bias_T();
	pSystem->FS_UpdateBiasImpulse(pJoint->m_ChildLink);
	pSystem->FS_UpdateDelVelocity();
}

void JointConstraint::_applyimpulse_B_Y(int /* _idx */)
{
	// 미구현
}

void JointConstraint::_applyimpulse_B_RP(int /* _idx */)
{
	// 미구현
}

void JointConstraint::ApplyImpulse(int _idx)
{
	(this->*m_pfn_applyimpulse)(_idx);
}

void JointConstraint::_getdelvelocity_R(sr_real * sjari)
{
	if (pSystem->m_bExcited)
		sjari[0] = m_pRstate->m_rDelVel;
	else
		sjari[0] = 0.0;
}

void JointConstraint::_getdelvelocity_P(sr_real * sjari)
{
	if (pSystem->m_bExcited)
		sjari[0] = m_pPstate->m_rDelVel;
	else
		sjari[0] = 0.0;
}

void JointConstraint::_getdelvelocity_U_1(sr_real * sjari)
{
	if (pSystem->m_bExcited)
		sjari[0] = m_pUstate->m_rDelVel[0];
	else
		sjari[0] = 0.0;
}

void JointConstraint::_getdelvelocity_U_2(sr_real * sjari)
{
	if (pSystem->m_bExcited)
		sjari[0] = m_pUstate->m_rDelVel[1];
	else
		sjari[0] = 0.0;
}

void JointConstraint::_getdelvelocity_B_Y(sr_real * /* sjari */)
{
	// 미구현
}

void JointConstraint::_getdelvelocity_B_RP(sr_real * /* sjari */)
{
	// 미구현
}

void JointConstraint::GetDelVelocity(sr_real * sjari)
{
	(this->*m_pfn_getdelvelocity)(sjari);
}

void JointConstraint::Excite()
{
	pSystem->ExciteSystem();
}

void JointConstraint::_unexcite_R()
{
	m_pRstate->m_rImp = 0.0;
}

void JointConstraint::_unexcite_P()
{
	m_pPstate->m_rImp = 0.0;
}

void JointConstraint::_unexcite_U_1()
{
	m_pUstate->m_rImp[0] = 0.0;
}

void JointConstraint::_unexcite_U_2()
{
	m_pUstate->m_rImp[1] = 0.0;
}

void JointConstraint::_unexcite_B_Y()
{
	// 미구현
}

void JointConstraint::_unexcite_B_RP()
{
	// 미구현
}

void JointConstraint::UnExcite()
{
	(this->*m_pfn_unexcite)();
	pSystem->UnExciteSystem();
}

void JointConstraint::_setimpulse_R(sr_real * _lambda)
{
	m_pRstate->m_rImp += _lambda[0];
}

void JointConstraint::_setimpulse_P(sr_real * _lambda)
{
	m_pPstate->m_rImp += _lambda[0];
}

void JointConstraint::_setimpulse_U_1(sr_real * _lambda)
{
	m_pUstate->m_rImp[0] += _lambda[0];
}

void JointConstraint::_setimpulse_U_2(sr_real * _lambda)
{
	m_pUstate->m_rImp[1] += _lambda[0];
}

void JointConstraint::_setimpulse_B_Y(sr_real * /* _lambda */)
{
	// 미구현
}

void JointConstraint::_setimpulse_B_RP(sr_real * /* _lambda */)
{
	// 미구현
}

void JointConstraint::SetImpulse(sr_real * _lambda)
{
	lambda = _lambda[0];
	(this->*m_pfn_setimpulse)(_lambda);
}

srSystem* JointConstraint::UF_Find_Constraint()
{
	return pSystem->m_UF_id;
}
//**********************************************************************//


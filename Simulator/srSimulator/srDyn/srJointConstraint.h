#pragma once

#include "srDyn/srConstraint.h"
#include "srDyn/srState.h"
#include "srDyn/srSystem.h"
#include "srDyn/srJoint.h"

//**********************************************************************//
// JointConstraint 
class JointConstraint: public Constraint
{
public:
	JointConstraint()
	{
		nd = 1;
		type1 = false;
	};

	// static memebers
	static void SetErp(sr_real _erp);
	static sr_real erp_jointpositionlimit;

	static void SetAllowedPenetration(sr_real _allowedpenetration);
	static sr_real allowedjointerror;

	static void SetBouncingThreshold(sr_real _bouncingthreshold);
	static sr_real bouncing_threshold;

	static void SetMaximumErpVelocity(sr_real _maximum_erp_velocity);
	static sr_real maximum_erp_velocity;

	static void	SetMaximumBouncingVelocity(sr_real _maximum_bouncing_velocity);
	static sr_real maximum_bouncing_velocity;

	//variables

	//=== PRESTEP ===//
	srJoint *	pJoint;
	srSystem *	pSystem;
	srJoint::ACTTYPE		actuationtype;
	srJoint::JOINTTYPE	jointtype;

	//-- Target JointState
	srRevoluteState  *	m_pRstate;
	srPrismaticState *	m_pPstate;
	srUniversalState *	m_pUstate;
	//srBallState		*	m_pBstate;


	//-- Limit
	sr_real	Limit[2];		// Position Limit  [0]:lower, [1]:upper
	sr_real	ForceLimit[2];	// Force Limit [0]:lower, [1]:upper


	//=== RUNTIME ===//
	sr_real	LimitError;
	sr_real	Negative_Velocity;

	bool	bActive;
	int		lifetime;
	sr_real	lambda;


	//-- Detection
	bool	Inspect_JointState();
	bool	(JointConstraint::*m_pfn_inspect_jointstate)();
	bool	_inspect_R_PositionLimit();
	bool	_inspect_R_TorqueLimit();

	bool	_inspect_P_PositionLimit();
	bool	_inspect_P_TorqueLimit();

	bool	_inspect_U1_PositionLimit();
	bool	_inspect_U1_TorqueLimit();
	bool	_inspect_U2_PositionLimit();
	bool	_inspect_U2_TorqueLimit();

	bool	_inspect_B_Yaw();
	bool	_inspect_B_RollPitch();

	// virtual function
	void	GetInformation(ConstraintInfo * info);
	void	(JointConstraint::*m_pfn_getInformation)(ConstraintInfo * info);
	void	_getInformation_PositionLimit(ConstraintInfo * info);
	void	_getInformation_TorqueLimit(ConstraintInfo * info);


	void	ApplyImpulse(int _idx);
	void	(JointConstraint::*m_pfn_applyimpulse)(int _idx);
	void	_applyimpulse_R(int _idx);
	void	_applyimpulse_P(int _idx);
	void	_applyimpulse_U_1(int _idx);
	void	_applyimpulse_U_2(int _idx);
	void	_applyimpulse_B_Y(int _idx);
	void	_applyimpulse_B_RP(int _idx);


	void	GetDelVelocity(sr_real * sjari);
	void	(JointConstraint::*m_pfn_getdelvelocity)(sr_real * sjari);
	void	_getdelvelocity_R(sr_real * sjari);
	void	_getdelvelocity_P(sr_real * sjari);
	void	_getdelvelocity_U_1(sr_real * sjari);
	void	_getdelvelocity_U_2(sr_real * sjari);
	void	_getdelvelocity_B_Y(sr_real * sjari);
	void	_getdelvelocity_B_RP(sr_real * sjari);


	void	Excite();

	void	UnExcite();
	void	(JointConstraint::*m_pfn_unexcite)();
	void	_unexcite_R();
	void	_unexcite_P();
	void	_unexcite_U_1();
	void	_unexcite_U_2();
	void	_unexcite_B_Y();
	void	_unexcite_B_RP();

	void	SetImpulse(sr_real * _lambda);
	void	(JointConstraint::*m_pfn_setimpulse)(sr_real * _lambda);
	void	_setimpulse_R(sr_real * _lambda);
	void	_setimpulse_P(sr_real * _lambda);
	void	_setimpulse_U_1(sr_real * _lambda);
	void	_setimpulse_U_2(sr_real * _lambda);
	void	_setimpulse_B_Y(sr_real * _lambda);
	void	_setimpulse_B_RP(sr_real * _lambda);

	srSystem*	UF_Find_Constraint();
};

#include "srDyn/srUniversalJoint.h"
#include "srDyn/srLink.h"
//#include "../srDyn/srState.h"

srUniversalJoint::srUniversalJoint()
: srJoint()
{
	m_Idx_State[0] = 0;
	m_Idx_State[1] = 1;

	m_Axis1 = se3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	m_Axis2 = se3(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

	m_rOffset[0] = 0.0;
	m_rOffset[1] = 0.0;

	m_rK[0]	= 0.0;
	m_rK[1] = 0.0;
	m_rC[0] = 0.01;
	m_rC[1] = 0.01;

	m_IsPosLimited[0] = false;
	m_IsPosLimited[1] = false;
	m_PosLimit_1[0] = -60;
	m_PosLimit_1[1] = 60;
	m_PosLimit_2[0] = -60;
	m_PosLimit_2[1] = 60;

	m_TorqueLimit_1[0] = -100;
	m_TorqueLimit_1[1] = 100;

	m_TorqueLimit_2[0] = -100;
	m_TorqueLimit_2[1] = 100;

	m_JointType = UNIVERSAL;
	
	GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
}

srUniversalJoint::~srUniversalJoint()
{
}

bool srUniversalJoint::IsPostionLimited(int idx /*= 0*/)
{
	if (idx == 0)
		return m_IsPosLimited[0];
	else if (idx == 1)
		return m_IsPosLimited[1];
	else
		return false;
}

void srUniversalJoint::MakePositionLimit(bool v /*= true*/, int idx /*= 3*/)
{
	if (idx == 0)
		m_IsPosLimited[0] = v;
	else if (idx == 1)
		m_IsPosLimited[1] = v;
	else if (idx == 2)
	{
		m_IsPosLimited[0] = v;
		m_IsPosLimited[1] = v;
	}
}

sr_real srUniversalJoint::GetPositionLowerLimit(int idx /*= 0*/)
{
	if (idx == 0)
		return m_PosLimit_1[0];
	else if (idx == 1)
		return m_PosLimit_2[0];
	else
		return 0.0;
}

sr_real srUniversalJoint::GetPositionUpperLimit(int idx /*= 0*/)
{
	if (idx == 0)
		return m_PosLimit_1[1];
	else if (idx == 1)
		return m_PosLimit_2[1];
	else
		return 0.0;
}

void srUniversalJoint::SetPositionLimit(sr_real lowerlimit, sr_real upperlimit, int idx /*= 2*/)
{
	if ( lowerlimit < upperlimit )
	{
		if (idx == 0)
		{
			m_PosLimit_1[0] = lowerlimit;
			m_PosLimit_1[1] = upperlimit;
		}
		else if (idx == 1)
		{
			m_PosLimit_2[0] = lowerlimit;
			m_PosLimit_2[1] = upperlimit;
		}
		else if (idx == 2)
		{
			m_PosLimit_1[0] = lowerlimit;
			m_PosLimit_1[1] = upperlimit;

			m_PosLimit_2[0] = lowerlimit;
			m_PosLimit_2[1] = upperlimit;
		}
	}
}
sr_real srUniversalJoint::GetTorqueLowerLimit(int idx /*= 0*/)
{
	if (idx == 0)
		return m_TorqueLimit_1[0];
	else if (idx == 1)
		return m_TorqueLimit_2[0];
	else
		return 0.0;
}

sr_real srUniversalJoint::GetTorqueUpperLimit(int idx /*= 0*/)
{
	if (idx == 0)
		return m_TorqueLimit_1[1];
	else if (idx == 1)
		return m_TorqueLimit_2[1];
	else
		return 0.0;
}

void srUniversalJoint::SetTorqueLimit(sr_real lowerlimit, sr_real upperlimit, int idx /*=2*/)
{
	if ( lowerlimit < upperlimit )
	{
		if (idx == 0)
		{
			m_TorqueLimit_1[0] = lowerlimit;
			m_TorqueLimit_1[1] = upperlimit;
		}
		else if (idx == 1)
		{
			m_TorqueLimit_2[0] = lowerlimit;
			m_TorqueLimit_2[1] = upperlimit;
		}
		else if (idx == 2)
		{
			m_TorqueLimit_1[0] = lowerlimit;
			m_TorqueLimit_1[1] = upperlimit;

			m_TorqueLimit_2[0] = lowerlimit;
			m_TorqueLimit_2[1] = upperlimit;
		}
	}
}

sr_real srUniversalJoint::GetOffset(int idx)
{
	if (idx == 0)
		return m_rOffset[0];
	else if (idx == 1)
		return m_rOffset[1];
	else
		return 0.0;
}

void srUniversalJoint::SetOffset(sr_real v, int idx)
{
	if (idx == 0)
		m_rOffset[0] = v;
	else if (idx == 1)
		m_rOffset[1] = v;
}

sr_real srUniversalJoint::GetSpringCoeff(int idx)
{
	if (idx == 0)
		return m_rK[0];
	else if (idx == 1)
		return m_rK[1];
	else
		return 0.0;
}

void srUniversalJoint::SetSpringCoeff(sr_real v, int idx)
{
	if (v >= 0.0)
	{
		if (idx == 0)
			m_rK[0] = v;
		else if (idx == 1)
			m_rK[1] = v;
		else if (idx == 2)
		{
			m_rK[0] = v;
			m_rK[1] = v;
		}
	}
}

sr_real srUniversalJoint::GetDampingCoeff(int idx)
{
	if (idx == 0)
		return m_rC[0];
	else if (idx == 1)
		return m_rC[1];
	else
		return 0.0;
}

void srUniversalJoint::SetDampingCoeff(sr_real v, int idx)
{
	if (v >= 0.0)
	{
		if (idx == 0)
			m_rC[0] = v;
		else if (idx == 1)
			m_rC[1] = v;
		else if (idx == 2)
		{
			m_rC[0] = v;
			m_rC[1] = v;
		}
	}
}

srUniversalState& srUniversalJoint::GetUniversalJointState()
{
	return m_State;
}

void srUniversalJoint::SetDeviceOnOff(bool onoff)
{
	if (HYBRID == m_ActType)
	{
		if ( true == onoff )
		{
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Hybrid;
		} 
		else // if ( false == onoff)
		{
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Passive;
		}
	}
}

srState* srUniversalJoint::GetStatePtr()
{
	return reinterpret_cast<srState*>(&m_State);
}

void srUniversalJoint::Initialize()
{
	switch(GetActType()) {
		default:
		case PASSIVE:
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Passive;
			break;
		case TORQUE:
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Torque;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Torque;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Torque;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Torque;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Torque;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Torque;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Torque;
			break;
		case VELOCITY:
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Servo;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Servo;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Servo;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Servo;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Servo;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Servo;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Servo;
			break;
		case HYBRID:
			m_pfnFS_UpdateForce		  = &srUniversalJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srUniversalJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srUniversalJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srUniversalJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srUniversalJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srUniversalJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srUniversalJoint::_FS_UpdateLocalAcc_Hybrid;
			break;
	}
}

inline void srUniversalJoint::FS_UpdateForce(const dse3& F)
{
	(this->*m_pfnFS_UpdateForce)(F);
}

inline void srUniversalJoint::FS_UpdateAIS_K(const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K)(AI);
}

inline void srUniversalJoint::FS_UpdateAIS_K_P(AInertia& __AI, const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K_P)(__AI, AI);
}

inline void srUniversalJoint::FS_UpdateBiasImp(dse3& Cias, const dse3& Bias)
{
	(this->*m_pfnFS_UpdateBiasImp)(Cias, Bias);
}

inline void srUniversalJoint::FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	(this->*m_pfnFS_UpdateBiasforce)(Cias, Bias, AI, V);
}

inline void srUniversalJoint::FS_UpdateLocalDelVel(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalDelVel)(jari, DV);
}

inline void srUniversalJoint::FS_UpdateLocalAcc(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalAcc)(jari, DV);
}

///////////////////////////////////////////////////////////////////////////////
// Passive
void srUniversalJoint::_FS_UpdateForce_Passive(const dse3& /* F */)
{
	// Do nothing
}

void srUniversalJoint::_FS_UpdateAIS_K_Passive(const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);
}

void srUniversalJoint::_FS_UpdateAIS_K_P_Passive(AInertia& __AI, const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);

	AI_temp2 = m_FS_AI_med;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_1 * m_FS_AIS_1, m_FS_AIS_1);
	__AI = AI_temp2.Transform(m_FS_SE3_2);
}

void srUniversalJoint::_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias)
{
	dse3 Cias_temp;

	m_FS_T2 = m_State.m_rImp[m_Idx_State[1]] - (Bias * m_FS_Screw2);
	Cias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias);

	m_FS_T1 = m_State.m_rImp[m_Idx_State[0]] - (Cias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Cias_temp);
}

void srUniversalJoint::_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[0][3] = - m_rK[0] * (m_State.m_rValue[0][0] - m_rOffset[0]) - m_rC[0] * m_State.m_rValue[0][1];
	m_State.m_rValue[1][3] = - m_rK[1] * (m_State.m_rValue[1][0] - m_rOffset[1]) - m_rC[1] * m_State.m_rValue[1][1];

	m_FS_W2 = ad(V,m_FS_LocalVelocity_2);
	Bias_temp = AI * m_FS_W2 + Bias;

	m_FS_T2 = m_State.m_rValue[m_Idx_State[1]][3] - (Bias_temp * m_FS_Screw2);
	Bias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias_temp);

	m_FS_W1 = ad(Ad(m_FS_SE3_2,(V - m_FS_LocalVelocity_2)),m_FS_LocalVelocity_1);
	Bias_temp = m_FS_AI_med * m_FS_W1 + Bias_temp;

	m_FS_T1 = m_State.m_rValue[m_Idx_State[0]][3] - (Bias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Bias_temp);
}

void srUniversalJoint::_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rDelVel[m_Idx_State[0]] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_State.m_rDelVel[m_Idx_State[0]] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rDelVel[m_Idx_State[1]] = m_FS_K_2 * m_FS_T2;
	jari = temp1 + (m_State.m_rDelVel[m_Idx_State[1]] * m_FS_Screw2);
}

void srUniversalJoint::_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rValue[m_Idx_State[0]][2] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_FS_W1 + m_State.m_rValue[m_Idx_State[0]][2] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rValue[m_Idx_State[1]][2] = m_FS_K_2 * m_FS_T2;
	jari  = temp1 + m_FS_W2 + m_State.m_rValue[m_Idx_State[1]][2] * m_FS_Screw2;
}

///////////////////////////////////////////////////////////////////////////////
// Torque
void srUniversalJoint::_FS_UpdateForce_Torque(const dse3& /* F */)
{
	// Do nothing
}

void srUniversalJoint::_FS_UpdateAIS_K_Torque(const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);
}

void srUniversalJoint::_FS_UpdateAIS_K_P_Torque(AInertia& __AI, const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);

	AI_temp2 = m_FS_AI_med;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_1 * m_FS_AIS_1, m_FS_AIS_1);
	__AI = AI_temp2.Transform(m_FS_SE3_2);
}

void srUniversalJoint::_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias)
{
	dse3 Cias_temp;

	m_FS_T2 = m_State.m_rImp[m_Idx_State[1]] - (Bias * m_FS_Screw2);
	Cias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias);

	m_FS_T1 = m_State.m_rImp[m_Idx_State[0]] - (Cias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Cias_temp);
}

void srUniversalJoint::_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[0][3] = m_State.m_rCommand[0];
	m_State.m_rValue[1][3] = m_State.m_rCommand[1];

	m_FS_W2 = ad(V,m_FS_LocalVelocity_2);
	Bias_temp = AI * m_FS_W2 + Bias;

	m_FS_T2 = m_State.m_rValue[m_Idx_State[1]][3] - (Bias_temp * m_FS_Screw2);
	Bias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias_temp);

	m_FS_W1 = ad(Ad(m_FS_SE3_2,(V - m_FS_LocalVelocity_2)),m_FS_LocalVelocity_1);
	Bias_temp = m_FS_AI_med * m_FS_W1 + Bias_temp;

	m_FS_T1 = m_State.m_rValue[m_Idx_State[0]][3] - (Bias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Bias_temp);
}

void srUniversalJoint::_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rDelVel[m_Idx_State[0]] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_State.m_rDelVel[m_Idx_State[0]] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rDelVel[m_Idx_State[1]] = m_FS_K_2 * m_FS_T2;
	jari = temp1 + (m_State.m_rDelVel[m_Idx_State[1]] * m_FS_Screw2);
}

void srUniversalJoint::_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rValue[m_Idx_State[0]][2] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_FS_W1 + m_State.m_rValue[m_Idx_State[0]][2] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rValue[m_Idx_State[1]][2] = m_FS_K_2 * m_FS_T2;
	jari  = temp1 + m_FS_W2 + m_State.m_rValue[m_Idx_State[1]][2] * m_FS_Screw2;
}

///////////////////////////////////////////////////////////////////////////////
// Servo
void srUniversalJoint::_FS_UpdateForce_Servo(const dse3& /* F */)
{
	// Do nothing
}

void srUniversalJoint::_FS_UpdateAIS_K_Servo(const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);
}

void srUniversalJoint::_FS_UpdateAIS_K_P_Servo(AInertia& __AI, const AInertia& AI)
{
	AInertia AI_temp2;

	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_K_2 = 1.0 / (m_FS_AIS_2 * m_FS_Screw2);

	AI_temp2 = AI;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_2 * m_FS_AIS_2, m_FS_AIS_2);

	m_FS_AI_med = AI_temp2.Transform(Inv(m_FS_SE3_2));

	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	m_FS_K_1 = 1.0 / (m_FS_AIS_1 *  m_FS_Screw1);

	AI_temp2 = m_FS_AI_med;
	AI_temp2.SubtractKroneckerProduct(m_FS_K_1 * m_FS_AIS_1, m_FS_AIS_1);
	__AI = AI_temp2.Transform(m_FS_SE3_2);
}

void srUniversalJoint::_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias)
{
	dse3 Cias_temp;

	m_FS_T2 = m_State.m_rImp[m_Idx_State[1]] - (Bias * m_FS_Screw2);
	Cias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias);

	m_FS_T1 = m_State.m_rImp[m_Idx_State[0]] - (Cias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Cias_temp);
}

void srUniversalJoint::_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	//m_State.m_rValue[0][3] = m_State.m_rCommand[0];
	//m_State.m_rValue[1][3] = m_State.m_rCommand[1];

	m_State.m_rValue[0][3] = 0.0;
	m_State.m_rValue[1][3] = 0.0;


	m_FS_W2 = ad(V,m_FS_LocalVelocity_2);
	Bias_temp = AI * m_FS_W2 + Bias;

	m_FS_T2 = m_State.m_rValue[m_Idx_State[1]][3] - (Bias_temp * m_FS_Screw2);
	Bias_temp = InvdAd(m_FS_SE3_2, (m_FS_K_2 * m_FS_T2 * m_FS_AIS_2) + Bias_temp);

	m_FS_W1 = ad(Ad(m_FS_SE3_2,(V - m_FS_LocalVelocity_2)),m_FS_LocalVelocity_1);
	Bias_temp = m_FS_AI_med * m_FS_W1 + Bias_temp;

	m_FS_T1 = m_State.m_rValue[m_Idx_State[0]][3] - (Bias_temp * m_FS_Screw1);
	Cias = dAd(m_FS_SE3_2, (m_FS_K_1 * m_FS_T1 * m_FS_AIS_1) + Bias_temp);
}

void srUniversalJoint::_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rDelVel[m_Idx_State[0]] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_State.m_rDelVel[m_Idx_State[0]] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rDelVel[m_Idx_State[1]] = m_FS_K_2 * m_FS_T2;
	jari = temp1 + (m_State.m_rDelVel[m_Idx_State[1]] * m_FS_Screw2);
}

void srUniversalJoint::_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV)
{
	se3 temp1;

	temp1 = Ad(m_FS_SE3_2, DV);

	m_FS_T1 -= temp1 * m_FS_AIS_1;
	m_State.m_rValue[m_Idx_State[0]][2] = m_FS_K_1 * m_FS_T1;
	temp1 = InvAd(m_FS_SE3_2, temp1 + m_FS_W1 + m_State.m_rValue[m_Idx_State[0]][2] * m_FS_Screw1);

	m_FS_T2 -= temp1 * m_FS_AIS_2;
	m_State.m_rValue[m_Idx_State[1]][2] = m_FS_K_2 * m_FS_T2;
	jari  = temp1 + m_FS_W2 + m_State.m_rValue[m_Idx_State[1]][2] * m_FS_Screw2;
}

///////////////////////////////////////////////////////////////////////////////
// Hybrid
void srUniversalJoint::_FS_UpdateForce_Hybrid(const dse3& F)
{
	m_State.m_rValue[m_Idx_State[0]][3] = InvAd(m_FS_SE3_2, m_FS_Screw1) * F;
	m_State.m_rValue[m_Idx_State[1]][3] = m_FS_Screw2 * F;
}

void srUniversalJoint::_FS_UpdateAIS_K_Hybrid(const AInertia& AI)
{
	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_AI_med = AI.Transform(Inv(m_FS_SE3_2));
	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
}

void srUniversalJoint::_FS_UpdateAIS_K_P_Hybrid(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS_2 = AI* m_FS_Screw2;
	m_FS_AI_med = AI.Transform(Inv(m_FS_SE3_2));
	m_FS_AIS_1 = m_FS_AI_med * m_FS_Screw1;
	__AI = AI;
}

void srUniversalJoint::_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias)
{
	m_State.m_rDelVel[0] = 0.0;
	m_State.m_rDelVel[1] = 0.0;

	Cias = Bias;
}

void srUniversalJoint::_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[0][2] = m_State.m_rCommand[0];
	m_State.m_rValue[1][2] = m_State.m_rCommand[1];

	m_FS_W2 = ad(V,m_FS_LocalVelocity_2);
	Bias_temp = AI * m_FS_W2 + Bias;

	Bias_temp = InvdAd(m_FS_SE3_2, (m_State.m_rValue[m_Idx_State[1]][2] * m_FS_AIS_2) + Bias_temp);

	m_FS_W1 = ad(Ad(m_FS_SE3_2,(V - m_FS_LocalVelocity_2)),m_FS_LocalVelocity_1);
	Bias_temp = m_FS_AI_med * m_FS_W1 + Bias_temp;

	Cias = dAd(m_FS_SE3_2, (m_State.m_rValue[m_Idx_State[0]][2] * m_FS_AIS_1) + Bias_temp);
}

void srUniversalJoint::_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV)
{
	jari = DV;

	//jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srUniversalJoint::_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV)
{
	se3 temp1 = InvAd(m_FS_SE3_2, Ad(m_FS_SE3_2, DV) + m_FS_W1 + m_State.m_rValue[m_Idx_State[0]][2] * m_FS_Screw1);
	jari  = temp1 + m_FS_W2 + m_State.m_rValue[m_Idx_State[1]][2] * m_FS_Screw2;
}
//////////////////////////////////////////////////////////////////////////
SE3& srUniversalJoint::FS_Transform()
{
	m_FS_SE3_1 = Exp(m_FS_Screw1,m_State.m_rValue[m_Idx_State[0]][0]);
	m_FS_SE3_2 = Exp(m_FS_Screw2,m_State.m_rValue[m_Idx_State[1]][0]);
	m_FS_SE3 = m_FS_SE3_1 * m_FS_SE3_2;
	return m_FS_SE3;
}
se3& srUniversalJoint::FS_UpdateLocalVelocity()
{
	m_FS_LocalVelocity_1 = m_State.m_rValue[m_Idx_State[0]][1] * m_FS_Screw1;
	m_FS_LocalVelocity_2 = m_State.m_rValue[m_Idx_State[1]][1] * m_FS_Screw2;
	m_FS_LocalVelocity = InvAd(m_FS_SE3_2, m_FS_LocalVelocity_1) + m_FS_LocalVelocity_2;
	return m_FS_LocalVelocity;
}
se3& srUniversalJoint::FS_UpdatePosErrorLocalVelocity()
{
	m_FS_LocalVelocity_1 = m_State.m_rPosErrVel[m_Idx_State[0]] * m_FS_Screw1;
	m_FS_LocalVelocity_2 = m_State.m_rPosErrVel[m_Idx_State[1]] * m_FS_Screw2;
	m_FS_LocalVelocity = InvAd(m_FS_SE3_2, m_FS_LocalVelocity_1) + m_FS_LocalVelocity_2;
	return m_FS_LocalVelocity;
}
void srUniversalJoint::FS_SetScrew(int i)
{
	if ( i == 0)
	{
		m_Idx_State[0] = 0;
		m_Idx_State[1] = 1;

		m_FS_Screw1 = Ad(m_ChildLinkToJoint, m_Axis1);
		m_FS_Screw2 = Ad(m_ChildLinkToJoint, m_Axis2);
	}
	else if ( i == 1 )
	{
		m_Idx_State[0] = 1;
		m_Idx_State[1] = 0;

		m_FS_Screw1 = Ad(m_ChildLinkToJoint, -m_Axis2);
		m_FS_Screw2 = Ad(m_ChildLinkToJoint, -m_Axis1);
	}
}
void srUniversalJoint::FS_ResetT()
{
	m_FS_T1 = 0.0;
	m_FS_T2 = 0.0;
}


#include "../srDyn/srPrismaticJoint.h"
#include "srDyn/srLink.h"
//#include "../srDyn/srState.h"

srPrismaticJoint::srPrismaticJoint()
: srJoint()
{
	m_Axis = se3(0.0,0.0,0.0,0.0,0.0,1.0);

	m_rOffset = 0.0;
	m_rK = 0.0;
	m_rC = 0.01;

	m_IsPosLimited = false;
	m_PosLimit[0] = -30;
	m_PosLimit[1] = 30;

	m_TorqueLimit[0] = -100;
	m_TorqueLimit[1] = 100;

	m_JointType = PRISMATIC;
	
	GetGeomInfo().SetShape(srGeometryInfo::BOX);
}

srPrismaticJoint::~srPrismaticJoint()
{
}

bool srPrismaticJoint::IsPostionLimited()
{
	return m_IsPosLimited;
}

void srPrismaticJoint::MakePositionLimit(bool v)
{
	m_IsPosLimited = v;
}

sr_real srPrismaticJoint::GetPositionLowerLimit()
{
	return m_PosLimit[0];
}

sr_real srPrismaticJoint::GetPositionUpperLimit()
{
	return m_PosLimit[1];
}

void srPrismaticJoint::SetPositionLimit(sr_real lowerlimit, sr_real upperlimit)
{
	if ( lowerlimit < upperlimit )
	{
		m_PosLimit[0] = lowerlimit;
		m_PosLimit[1] = upperlimit;
	}
}
sr_real srPrismaticJoint::GetTorqueLowerLimit()
{
	return m_TorqueLimit[0];
}

sr_real srPrismaticJoint::GetTorqueUpperLimit()
{
	return m_TorqueLimit[1];
}

void srPrismaticJoint::SetTorqueLimit(sr_real lowerlimit, sr_real upperlimit)
{
	if ( lowerlimit < upperlimit )
	{
		m_TorqueLimit[0] = lowerlimit;
		m_TorqueLimit[1] = upperlimit;
	}
}

sr_real srPrismaticJoint::GetOffset()
{
	return m_rOffset;
}

void srPrismaticJoint::SetOffset(sr_real v)
{
	m_rOffset = v;
}

sr_real srPrismaticJoint::GetSpringCoeff()
{
	return m_rK;
}

void srPrismaticJoint::SetSpringCoeff(sr_real v)
{
	if (v >= 0.0)
		m_rK = v;
}

sr_real srPrismaticJoint::GetDampingCoeff()
{
	return m_rC;
}

void srPrismaticJoint::SetDampingCoeff(sr_real v)
{
	if (v >= 0.0)
		m_rC = v;
}

srPrismaticState& srPrismaticJoint::GetPrismaticJointState()
{
	return m_State;
}


void srPrismaticJoint::SetDeviceOnOff(bool onoff)
{
	if (HYBRID == m_ActType)
	{
		if ( true == onoff )
		{
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Hybrid;
		} 
		else // if ( false == onoff)
		{
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Passive;
		}
	}
}

srState* srPrismaticJoint::GetStatePtr()
{
	return reinterpret_cast<srState*>(&m_State);
}


void srPrismaticJoint::Initialize()
{
	switch(GetActType()) {
		default:
		case PASSIVE:
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Passive;
			break;
		case TORQUE:
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Torque;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Torque;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Torque;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Torque;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Torque;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Torque;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Torque;
			break;
		case VELOCITY:
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Servo;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Servo;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Servo;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Servo;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Servo;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Servo;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Servo;
			break;
		case HYBRID:
			m_pfnFS_UpdateForce		  = &srPrismaticJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srPrismaticJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srPrismaticJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srPrismaticJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srPrismaticJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srPrismaticJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srPrismaticJoint::_FS_UpdateLocalAcc_Hybrid;
			break;
	}
}

inline void srPrismaticJoint::FS_UpdateForce(const dse3& F)
{
	(this->*m_pfnFS_UpdateForce)(F);
}

inline void srPrismaticJoint::FS_UpdateAIS_K(const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K)(AI);
}

inline void srPrismaticJoint::FS_UpdateAIS_K_P(AInertia& __AI, const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K_P)(__AI, AI);
}

inline void srPrismaticJoint::FS_UpdateBiasImp(dse3& Cias, const dse3& Bias)
{
	(this->*m_pfnFS_UpdateBiasImp)(Cias, Bias);
}

inline void srPrismaticJoint::FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	(this->*m_pfnFS_UpdateBiasforce)(Cias, Bias, AI, V);
}

inline void srPrismaticJoint::FS_UpdateLocalDelVel(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalDelVel)(jari, DV);
}

inline void srPrismaticJoint::FS_UpdateLocalAcc(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalAcc)(jari, DV);
}

///////////////////////////////////////////////////////////////////////////////
// Passive
void srPrismaticJoint::_FS_UpdateForce_Passive(const dse3& /* F */)
{
	// Do nothing
}

void srPrismaticJoint::_FS_UpdateAIS_K_Passive(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srPrismaticJoint::_FS_UpdateAIS_K_P_Passive(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srPrismaticJoint::_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srPrismaticJoint::_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[3] = -m_rK * (m_State.m_rValue[0] - m_rOffset) - m_rC * m_State.m_rValue[1];

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srPrismaticJoint::_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srPrismaticJoint::_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Torque
void srPrismaticJoint::_FS_UpdateForce_Torque(const dse3& /* F */)
{
	// Do nothing
}

void srPrismaticJoint::_FS_UpdateAIS_K_Torque(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srPrismaticJoint::_FS_UpdateAIS_K_P_Torque(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srPrismaticJoint::_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srPrismaticJoint::_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[3] = m_State.m_rCommand;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srPrismaticJoint::_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srPrismaticJoint::_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Servo
void srPrismaticJoint::_FS_UpdateForce_Servo(const dse3& /* F */)
{
	// Do nothing
}

void srPrismaticJoint::_FS_UpdateAIS_K_Servo(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srPrismaticJoint::_FS_UpdateAIS_K_P_Servo(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srPrismaticJoint::_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srPrismaticJoint::_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	//	m_State.m_rValue[3] = m_State.m_rCommand;
	m_State.m_rValue[3] = 0.0;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srPrismaticJoint::_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srPrismaticJoint::_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Hybrid
void srPrismaticJoint::_FS_UpdateForce_Hybrid(const dse3& F)
{
	m_State.m_rValue[3] = m_FS_Screw * F;
}

void srPrismaticJoint::_FS_UpdateAIS_K_Hybrid(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
}

void srPrismaticJoint::_FS_UpdateAIS_K_P_Hybrid(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	__AI = AI;
}

void srPrismaticJoint::_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias)
{
	m_State.m_rDelVel = 0.0;
	Cias = Bias;

	//Cias = m_State.m_rDelVel * m_FS_AIS + Bias;
}

void srPrismaticJoint::_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[2] = m_State.m_rCommand;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Cias = m_State.m_rValue[2] * m_FS_AIS + Bias_temp;
}

void srPrismaticJoint::_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV)
{
	jari = DV;

	//jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srPrismaticJoint::_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV)
{
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}
//////////////////////////////////////////////////////////////////////////
SE3& srPrismaticJoint::FS_Transform()
{
	m_FS_SE3 = Exp(m_FS_Screw, m_State.m_rValue[0]);
	return m_FS_SE3;
}
se3& srPrismaticJoint::FS_UpdateLocalVelocity()
{
	m_FS_LocalVelocity = m_FS_Screw * m_State.m_rValue[1];
	return m_FS_LocalVelocity;
}
se3& srPrismaticJoint::FS_UpdatePosErrorLocalVelocity()
{
	m_FS_LocalVelocity = m_FS_Screw * m_State.m_rPosErrVel;
	return m_FS_LocalVelocity;
}
void srPrismaticJoint::FS_SetScrew(int i)
{
	if ( i == 0)
	{
		m_FS_Screw = Ad(m_ChildLinkToJoint, m_Axis);
	}
	else if ( i == 1 )
	{
		m_FS_Screw = Ad(m_ChildLinkToJoint, -m_Axis);
	}
}
void srPrismaticJoint::FS_ResetT()
{
	m_FS_T = 0.0;
}

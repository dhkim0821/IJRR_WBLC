#include "srDyn/srRevoluteJoint.h"
#include "srDyn/srLink.h"
//#include "../srDyn/srState.h"

srRevoluteJoint::srRevoluteJoint()
: srJoint()
{
	m_IsPosLimited = true;
	m_PosLimit[0] = -180;
	m_PosLimit[1] = 180;


	m_TorqueLimit[0] = -200;
	m_TorqueLimit[1] = 200;

	m_rOffset = 0.0;
	m_rK = 0.0;
	m_rC = 0.01;


	m_JointType = REVOLUTE;
	m_Axis = se3(0.0,0.0,1.0,0.0,0.0,0.0);
	
	GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
}

srRevoluteJoint::~srRevoluteJoint()
{
}

bool srRevoluteJoint::IsPostionLimited()
{
	return m_IsPosLimited;
}

void srRevoluteJoint::MakePositionLimit(bool v)
{
	m_IsPosLimited = v;
}

sr_real srRevoluteJoint::GetPositionLowerLimit()
{
	return m_PosLimit[0];
}

sr_real srRevoluteJoint::GetPositionUpperLimit()
{
	return m_PosLimit[1];
}

void srRevoluteJoint::SetPositionLimit(sr_real lowerlimit, sr_real upperlimit)
{
	if ( lowerlimit < upperlimit )
	{
		m_PosLimit[0] = lowerlimit;
		m_PosLimit[1] = upperlimit;
	}
}
sr_real srRevoluteJoint::GetTorqueLowerLimit()
{
	return m_TorqueLimit[0];
}

sr_real srRevoluteJoint::GetTorqueUpperLimit()
{
	return m_TorqueLimit[1];
}

void srRevoluteJoint::SetTorqueLimit(sr_real lowerlimit, sr_real upperlimit)
{
	if ( lowerlimit < upperlimit )
	{
		m_TorqueLimit[0] = lowerlimit;
		m_TorqueLimit[1] = upperlimit;
	}
}

sr_real srRevoluteJoint::GetOffset()
{
	return m_rOffset;
}

void srRevoluteJoint::SetOffset(sr_real v)
{
	m_rOffset = v;
}

sr_real srRevoluteJoint::GetSpringCoeff()
{
	return m_rK;
}

void srRevoluteJoint::SetSpringCoeff(sr_real v)
{
	if (v >= 0.0)
		m_rK = v;
}

sr_real srRevoluteJoint::GetDampingCoeff()
{
	return m_rC;
}

void srRevoluteJoint::SetDampingCoeff(sr_real v)
{
	if (v >= 0.0)
		m_rC = v;
}

srRevoluteState& srRevoluteJoint::GetRevoluteJointState()
{
	return m_State;
}

void srRevoluteJoint::SetDeviceOnOff(bool onoff)
{
//	if (HYBRID == m_ActType)
	{
		if ( true == onoff )
		{
			Initialize();
//			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Hybrid;
//			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Hybrid;
//			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Hybrid;
//			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Hybrid;
//			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Hybrid;
//			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Hybrid;
//			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Hybrid;
		} 
		else // if ( false == onoff)
		{
			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Passive;
		}
	}
}

srState* srRevoluteJoint::GetStatePtr()
{
	return reinterpret_cast<srState*>(&m_State);
}

void srRevoluteJoint::Initialize()
{
	switch(GetActType()) {
		default:
		case PASSIVE:
			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Passive;
			break;
		case TORQUE:
			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Torque;
			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Torque;
			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Torque;
			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Torque;
			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Torque;
			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Torque;
			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Torque;
			break;
		case VELOCITY:
			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Servo;
			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Servo;
			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Servo;
			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Servo;
			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Servo;
			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Servo;
			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Servo;
			break;
		case HYBRID:
			m_pfnFS_UpdateForce		  = &srRevoluteJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srRevoluteJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srRevoluteJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srRevoluteJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srRevoluteJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srRevoluteJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srRevoluteJoint::_FS_UpdateLocalAcc_Hybrid;
			break;
	}
}

inline void srRevoluteJoint::FS_UpdateForce(const dse3& F)
{
	(this->*m_pfnFS_UpdateForce)(F);
}

inline void srRevoluteJoint::FS_UpdateAIS_K(const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K)(AI);
}

inline void srRevoluteJoint::FS_UpdateAIS_K_P(AInertia& __AI, const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K_P)(__AI, AI);
}

inline void srRevoluteJoint::FS_UpdateBiasImp(dse3& Cias, const dse3& Bias)
{
	(this->*m_pfnFS_UpdateBiasImp)(Cias, Bias);
}

inline void srRevoluteJoint::FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	(this->*m_pfnFS_UpdateBiasforce)(Cias, Bias, AI, V);
}

inline void srRevoluteJoint::FS_UpdateLocalDelVel(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalDelVel)(jari, DV);
}

inline void srRevoluteJoint::FS_UpdateLocalAcc(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalAcc)(jari, DV);
}

///////////////////////////////////////////////////////////////////////////////
// Passive
void srRevoluteJoint::_FS_UpdateForce_Passive(const dse3& /* F */)
{
	// Do nothing
}

void srRevoluteJoint::_FS_UpdateAIS_K_Passive(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srRevoluteJoint::_FS_UpdateAIS_K_P_Passive(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srRevoluteJoint::_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srRevoluteJoint::_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[3] = -m_rK * (m_State.m_rValue[0] - m_rOffset) - m_rC * m_State.m_rValue[1];

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srRevoluteJoint::_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srRevoluteJoint::_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Torque
void srRevoluteJoint::_FS_UpdateForce_Torque(const dse3& /* F */)
{
	// Do nothing
}

void srRevoluteJoint::_FS_UpdateAIS_K_Torque(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srRevoluteJoint::_FS_UpdateAIS_K_P_Torque(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srRevoluteJoint::_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srRevoluteJoint::_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[3] = m_State.m_rCommand;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srRevoluteJoint::_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srRevoluteJoint::_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Servo
void srRevoluteJoint::_FS_UpdateForce_Servo(const dse3& /* F */)
{
	// Do nothing
}

void srRevoluteJoint::_FS_UpdateAIS_K_Servo(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
}

void srRevoluteJoint::_FS_UpdateAIS_K_P_Servo(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	m_FS_K = 1.0 / (m_FS_AIS * m_FS_Screw);
	__AI = AI;
	__AI.SubtractKroneckerProduct(m_FS_K * m_FS_AIS, m_FS_AIS);
}

void srRevoluteJoint::_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias)
{
	//m_FS_T = - Bias * m_FS_Screw;

	m_FS_T = m_State.m_rImp - (Bias * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias;
}

void srRevoluteJoint::_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	//	m_State.m_rValue[3] = m_State.m_rCommand;
	m_State.m_rValue[3] = 0.0;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	m_FS_T = m_State.m_rValue[3] - (Bias_temp * m_FS_Screw);
	Cias = m_FS_K * m_FS_T * m_FS_AIS + Bias_temp;
}

void srRevoluteJoint::_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rDelVel = m_FS_K * m_FS_T;
	jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srRevoluteJoint::_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV)
{
	m_FS_T -= DV * m_FS_AIS;
	m_State.m_rValue[2] = m_FS_K * m_FS_T;
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}

///////////////////////////////////////////////////////////////////////////////
// Hybrid
void srRevoluteJoint::_FS_UpdateForce_Hybrid(const dse3& F)
{
	m_State.m_rValue[3] = m_FS_Screw * F;
}

void srRevoluteJoint::_FS_UpdateAIS_K_Hybrid(const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
}

void srRevoluteJoint::_FS_UpdateAIS_K_P_Hybrid(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS = AI * m_FS_Screw;
	__AI = AI;
}

void srRevoluteJoint::_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias)
{
	m_State.m_rDelVel = 0.0;
	Cias = Bias;

	//Cias = m_State.m_rDelVel * m_FS_AIS + Bias;
}

void srRevoluteJoint::_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_rValue[2] = m_State.m_rCommand;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Cias = m_State.m_rValue[2] * m_FS_AIS + Bias_temp;
}

void srRevoluteJoint::_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV)
{
	jari = DV;

	//jari = DV + m_State.m_rDelVel * m_FS_Screw;
}

void srRevoluteJoint::_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV)
{
	jari = DV + m_FS_W + m_State.m_rValue[2] * m_FS_Screw;
}
//////////////////////////////////////////////////////////////////////////
SE3& srRevoluteJoint::FS_Transform()
{
	m_FS_SE3 = Exp(m_FS_Screw, m_State.m_rValue[0]);
	return m_FS_SE3;
}
se3& srRevoluteJoint::FS_UpdateLocalVelocity()
{
	m_FS_LocalVelocity = m_FS_Screw * m_State.m_rValue[1];
	return m_FS_LocalVelocity;
}
se3& srRevoluteJoint::FS_UpdatePosErrorLocalVelocity()
{
	m_FS_LocalVelocity = m_FS_Screw * m_State.m_rPosErrVel;
	return m_FS_LocalVelocity;
}
void srRevoluteJoint::FS_SetScrew(int i)
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
void srRevoluteJoint::FS_ResetT()
{
	m_FS_T = 0.0;
}


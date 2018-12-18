#include "srDyn/srBallJoint.h"
#include "srDyn/srLink.h"
//#include "../srDyn/srState.h"

srBallJoint::srBallJoint()
: srJoint()
{
	m_IsPosLimited[0] = false;
	m_IsPosLimited[1] = false;

	m_PosLimit_1 = 60;
	m_PosLimit_2[0] = -60;
	m_PosLimit_2[1] = 60;


	m_TorqueLimit[0][0] = -100;
	m_TorqueLimit[0][1] = 100;

	m_TorqueLimit[1][0] = -100;
	m_TorqueLimit[1][1] = 100;

	m_TorqueLimit[2][0] = -100;
	m_TorqueLimit[2][1] = 100;



	m_Offset = SO3();

	m_rK = 0.0;

	m_rC = 0.01;




	m_JointType = BALL;
	m_Axis[0] = se3(1.0,0.0,0.0,0.0,0.0,0.0);
	m_Axis[1] = se3(0.0,1.0,0.0,0.0,0.0,0.0);
	m_Axis[2] = se3(0.0,0.0,1.0,0.0,0.0,0.0);

	m_pfnFS_Transfrom = &srBallJoint::_FS_Transform_Straight;
	
	GetGeomInfo().SetShape(srGeometryInfo::SPHERE);	
}

srBallJoint::~srBallJoint()
{
}

srBallState& srBallJoint::GetBallJointState()
{
	return m_State;
}

void srBallJoint::SetDeviceOnOff(bool onoff)
{
	if (HYBRID == m_ActType)
	{
		if ( true == onoff )
		{
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Hybrid;
		} 
		else // if ( false == onoff)
		{
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Passive;
		}
	}
}

srState* srBallJoint::GetStatePtr()
{
	return reinterpret_cast<srState*>(&m_State);
}

void srBallJoint::Initialize()
{
	switch(GetActType()) {
		default:
		case PASSIVE:
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Passive;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Passive;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Passive;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Passive;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Passive;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Passive;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Passive;
			break;
		case TORQUE:
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Torque;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Torque;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Torque;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Torque;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Torque;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Torque;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Torque;
			break;
		case VELOCITY:
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Servo;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Servo;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Servo;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Servo;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Servo;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Servo;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Servo;
			break;
		case HYBRID:
			m_pfnFS_UpdateForce		  = &srBallJoint::_FS_UpdateForce_Hybrid;
			m_pfnFS_UpdateAIS_K		  = &srBallJoint::_FS_UpdateAIS_K_Hybrid;
			m_pfnFS_UpdateAIS_K_P	  = &srBallJoint::_FS_UpdateAIS_K_P_Hybrid;
			m_pfnFS_UpdateBiasImp	  = &srBallJoint::_FS_UpdateBiasImp_Hybrid;
			m_pfnFS_UpdateBiasforce	  = &srBallJoint::_FS_UpdateBiasforce_Hybrid;
			m_pfnFS_UpdateLocalDelVel = &srBallJoint::_FS_UpdateLocalDelVel_Hybrid;
			m_pfnFS_UpdateLocalAcc	  = &srBallJoint::_FS_UpdateLocalAcc_Hybrid;
			break;
	}
}

inline void srBallJoint::FS_UpdateForce(const dse3& F)
{
	(this->*m_pfnFS_UpdateForce)(F);
}

inline void srBallJoint::FS_UpdateAIS_K(const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K)(AI);
}

inline void srBallJoint::FS_UpdateAIS_K_P(AInertia& __AI, const AInertia& AI)
{
	(this->*m_pfnFS_UpdateAIS_K_P)(__AI, AI);
}

inline void srBallJoint::FS_UpdateBiasImp(dse3& Cias, const dse3& Bias)
{
	(this->*m_pfnFS_UpdateBiasImp)(Cias, Bias);
}

inline void srBallJoint::FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	(this->*m_pfnFS_UpdateBiasforce)(Cias, Bias, AI, V);
}

inline void srBallJoint::FS_UpdateLocalDelVel(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalDelVel)(jari, DV);
}

inline void srBallJoint::FS_UpdateLocalAcc(se3& jari, const se3& DV)
{
	(this->*m_pfnFS_UpdateLocalAcc)(jari, DV);
}

void Inv3by3_In_BallJoint(sr_real* x, sr_real* b)	// x * b = I
{
	sr_real a11 = x[4]*x[8]-x[5]*x[7];
	sr_real a12 = x[1]*x[8]-x[2]*x[7];
	sr_real a13 = x[1]*x[5]-x[2]*x[4];
	sr_real det = x[0]*a11-x[3]*a12+x[6]*a13;
	det = 1.0/det;

	b[0]= a11*det;	b[3]=-(x[3]*x[8]-x[5]*x[6])*det;	b[6]= (x[3]*x[7]-x[4]*x[6])*det;
	b[1]=-a12*det;	b[4]= (x[0]*x[8]-x[2]*x[6])*det;	b[7]=-(x[0]*x[7]-x[1]*x[6])*det;
	b[2]= a13*det;	b[5]=-(x[0]*x[5]-x[2]*x[3])*det;	b[8]= (x[0]*x[4]-x[1]*x[3])*det;
}

void MultMV3by3_In_BallJoint(sr_real*a, sr_real*v, sr_real*r)		// a*v=r, a: 3 by 3 matrix, v: 3-dim vector, r:3-dim vector
{
	r[0]=a[0]*v[0]+a[3]*v[1]+a[6]*v[2];
	r[1]=a[1]*v[0]+a[4]*v[1]+a[7]*v[2];
	r[2]=a[2]*v[0]+a[5]*v[1]+a[8]*v[2];
}

///////////////////////////////////////////////////////////////////////////////
// Passive
void srBallJoint::_FS_UpdateForce_Passive(const dse3& /* F */)
{
	// Do nothing
}

void srBallJoint::_FS_UpdateAIS_K_Passive(const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);

	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
					+ m_Pinv[1] * m_FS_AIS[1]
					+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
					+ m_Pinv[4] * m_FS_AIS[1]
					+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
					+ m_Pinv[7] * m_FS_AIS[1]
					+ m_Pinv[8] * m_FS_AIS[2];
}

void srBallJoint::_FS_UpdateAIS_K_P_Passive(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);

	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
					+ m_Pinv[1] * m_FS_AIS[1]
					+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
					+ m_Pinv[4] * m_FS_AIS[1]
					+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
					+ m_Pinv[7] * m_FS_AIS[1]
					+ m_Pinv[8] * m_FS_AIS[2];

	__AI = AI;

	__AI.SubtractKroneckerProduct(m_FS_AISPinv[0],m_FS_AIS[0]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[1],m_FS_AIS[1]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[2],m_FS_AIS[2]);
}

void srBallJoint::_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias)
{
	Vec3 tmp;

	tmp[0] = m_FS_Screw[0] * Bias;
	tmp[1] = m_FS_Screw[1] * Bias;
	tmp[2] = m_FS_Screw[2] * Bias;

	m_FS_T = m_State.m_Imp - tmp;

	Cias = Bias
		 + m_FS_AISPinv[0] * m_FS_T[0]
		 + m_FS_AISPinv[1] * m_FS_T[1]
		 + m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_Torque = - m_rK * Log(m_State.m_SO3Pos * Inv(m_Offset)) - m_rC * m_State.m_Vel;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Vec3 tmp;

	tmp[0] = m_FS_Screw[0] * Bias_temp;
	tmp[1] = m_FS_Screw[1] * Bias_temp;
	tmp[2] = m_FS_Screw[2] * Bias_temp;

	m_FS_T = m_State.m_Torque - tmp;

	Cias = Bias_temp
		 + m_FS_AISPinv[0] * m_FS_T[0]
		 + m_FS_AISPinv[1] * m_FS_T[1]
		 + m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];

	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_DelVel[0]) );

	jari = DV
		 + m_FS_Screw[0] * m_State.m_DelVel[0]
		 + m_FS_Screw[1] * m_State.m_DelVel[1]
		 + m_FS_Screw[2] * m_State.m_DelVel[2];
}

void srBallJoint::_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];

	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_Acc[0]) );

	jari = DV + m_FS_W
		 + m_FS_Screw[0] * m_State.m_Acc[0]
		 + m_FS_Screw[1] * m_State.m_Acc[1]
		 + m_FS_Screw[2] * m_State.m_Acc[2];
}

///////////////////////////////////////////////////////////////////////////////
// Torque
void srBallJoint::_FS_UpdateForce_Torque(const dse3& /* F */)
{
	// Do nothing
}

void srBallJoint::_FS_UpdateAIS_K_Torque(const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);


	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
	+ m_Pinv[1] * m_FS_AIS[1]
	+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
	+ m_Pinv[4] * m_FS_AIS[1]
	+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
	+ m_Pinv[7] * m_FS_AIS[1]
	+ m_Pinv[8] * m_FS_AIS[2];
}

void srBallJoint::_FS_UpdateAIS_K_P_Torque(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);


	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
					+ m_Pinv[1] * m_FS_AIS[1]
					+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
					+ m_Pinv[4] * m_FS_AIS[1]
					+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
					+ m_Pinv[7] * m_FS_AIS[1]
					+ m_Pinv[8] * m_FS_AIS[2];

	__AI = AI;

	__AI.SubtractKroneckerProduct(m_FS_AISPinv[0],m_FS_AIS[0]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[1],m_FS_AIS[1]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[2],m_FS_AIS[2]);
}

void srBallJoint::_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias)
{
	Vec3 tmp;

	tmp[0] = m_FS_Screw[0] * Bias;
	tmp[1] = m_FS_Screw[1] * Bias;
	tmp[2] = m_FS_Screw[2] * Bias;

	m_FS_T = m_State.m_Imp - tmp;

	Cias = Bias
		+ m_FS_AISPinv[0] * m_FS_T[0]
	+ m_FS_AISPinv[1] * m_FS_T[1]
	+ m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_Torque = m_State.m_Command;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Vec3 tmp;

	tmp[0] = m_FS_Screw[0] * Bias_temp;
	tmp[1] = m_FS_Screw[1] * Bias_temp;
	tmp[2] = m_FS_Screw[2] * Bias_temp;

	m_FS_T = m_State.m_Torque - tmp;

	Cias = Bias_temp
		+ m_FS_AISPinv[0] * m_FS_T[0]
	+ m_FS_AISPinv[1] * m_FS_T[1]
	+ m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];


	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_DelVel[0]) );

	jari = DV
		 + m_FS_Screw[0] * m_State.m_DelVel[0]
		 + m_FS_Screw[1] * m_State.m_DelVel[1]
		 + m_FS_Screw[2] * m_State.m_DelVel[2];
}

void srBallJoint::_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];


	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_Acc[0]) );

	jari = DV + m_FS_W
		 + m_FS_Screw[0] * m_State.m_Acc[0]
		 + m_FS_Screw[1] * m_State.m_Acc[1]
		 + m_FS_Screw[2] * m_State.m_Acc[2];
}

///////////////////////////////////////////////////////////////////////////////
// Servo
void srBallJoint::_FS_UpdateForce_Servo(const dse3& /* F */)
{
	// Do nothing
}

void srBallJoint::_FS_UpdateAIS_K_Servo(const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);


	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
	+ m_Pinv[1] * m_FS_AIS[1]
	+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
	+ m_Pinv[4] * m_FS_AIS[1]
	+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
	+ m_Pinv[7] * m_FS_AIS[1]
	+ m_Pinv[8] * m_FS_AIS[2];
}

void srBallJoint::_FS_UpdateAIS_K_P_Servo(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	m_P[0] = m_FS_Screw[0] * m_FS_AIS[0];
	m_P[1] = m_FS_Screw[1] * m_FS_AIS[0];
	m_P[2] = m_FS_Screw[2] * m_FS_AIS[0];

	m_P[3] = m_FS_Screw[0] * m_FS_AIS[1];
	m_P[4] = m_FS_Screw[1] * m_FS_AIS[1];
	m_P[5] = m_FS_Screw[2] * m_FS_AIS[1];

	m_P[6] = m_FS_Screw[0] * m_FS_AIS[2];
	m_P[7] = m_FS_Screw[1] * m_FS_AIS[2];
	m_P[8] = m_FS_Screw[2] * m_FS_AIS[2];

	Inv3by3_In_BallJoint(m_P, m_Pinv);


	m_FS_AISPinv[0] = m_Pinv[0] * m_FS_AIS[0]
					+ m_Pinv[1] * m_FS_AIS[1]
					+ m_Pinv[2] * m_FS_AIS[2];

	m_FS_AISPinv[1] = m_Pinv[3] * m_FS_AIS[0]
					+ m_Pinv[4] * m_FS_AIS[1]
					+ m_Pinv[5] * m_FS_AIS[2];

	m_FS_AISPinv[2] = m_Pinv[6] * m_FS_AIS[0]
					+ m_Pinv[7] * m_FS_AIS[1]
					+ m_Pinv[8] * m_FS_AIS[2];

	__AI = AI;

	__AI.SubtractKroneckerProduct(m_FS_AISPinv[0],m_FS_AIS[0]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[1],m_FS_AIS[1]);
	__AI.SubtractKroneckerProduct(m_FS_AISPinv[2],m_FS_AIS[2]);
}

void srBallJoint::_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias)
{
	Vec3 tmp;

	tmp[0] = m_FS_Screw[0] * Bias;
	tmp[1] = m_FS_Screw[1] * Bias;
	tmp[2] = m_FS_Screw[2] * Bias;

	m_FS_T = m_State.m_Imp - tmp;

	Cias = Bias
		 + m_FS_AISPinv[0] * m_FS_T[0]
		 + m_FS_AISPinv[1] * m_FS_T[1]
		 + m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_Torque = 0.0;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Vec3 tmp;
	
	tmp[0] = m_FS_Screw[0] * Bias_temp;
	tmp[1] = m_FS_Screw[1] * Bias_temp;
	tmp[2] = m_FS_Screw[2] * Bias_temp;

	m_FS_T = m_State.m_Torque - tmp;

	Cias = Bias_temp
		 + m_FS_AISPinv[0] * m_FS_T[0]
		 + m_FS_AISPinv[1] * m_FS_T[1]
		 + m_FS_AISPinv[2] * m_FS_T[2];
}

void srBallJoint::_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];


	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_DelVel[0]) );

	jari = DV
		+ m_FS_Screw[0] * m_State.m_DelVel[0]
		+ m_FS_Screw[1] * m_State.m_DelVel[1]
		+ m_FS_Screw[2] * m_State.m_DelVel[2];
}

void srBallJoint::_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV)
{
	m_FS_T[0] -= DV * m_FS_AIS[0];
	m_FS_T[1] -= DV * m_FS_AIS[1];
	m_FS_T[2] -= DV * m_FS_AIS[2];


	MultMV3by3_In_BallJoint( m_Pinv, &(m_FS_T[0]), &(m_State.m_Acc[0]) );

	jari = DV + m_FS_W
		+ m_FS_Screw[0] * m_State.m_Acc[0]
		+ m_FS_Screw[1] * m_State.m_Acc[1]
		+ m_FS_Screw[2] * m_State.m_Acc[2];
}

///////////////////////////////////////////////////////////////////////////////
// Hybrid
void srBallJoint::_FS_UpdateForce_Hybrid(const dse3& F)
{
	m_State.m_Torque[0] = m_FS_Screw[0] * F;
	m_State.m_Torque[1] = m_FS_Screw[1] * F;
	m_State.m_Torque[2] = m_FS_Screw[2] * F;
}

void srBallJoint::_FS_UpdateAIS_K_Hybrid(const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];
}

void srBallJoint::_FS_UpdateAIS_K_P_Hybrid(AInertia& __AI, const AInertia& AI)
{
	m_FS_AIS[0] = AI * m_FS_Screw[0];
	m_FS_AIS[1] = AI * m_FS_Screw[1];
	m_FS_AIS[2] = AI * m_FS_Screw[2];

	__AI = AI;
}

void srBallJoint::_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias)
{
	m_State.m_DelVel = 0.0;
	Cias = Bias;
}

void srBallJoint::_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V)
{
	dse3 Bias_temp;

	m_State.m_Acc = m_State.m_Command;

	m_FS_W = ad(V, m_FS_LocalVelocity);
	Bias_temp = AI * m_FS_W + Bias;

	Cias = Bias_temp
		 + m_FS_AIS[0] * m_State.m_Acc[0]
		 + m_FS_AIS[1] * m_State.m_Acc[1]
		 + m_FS_AIS[2] * m_State.m_Acc[2];
}

void srBallJoint::_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV)
{
	jari = DV;
}

void srBallJoint::_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV)
{
	jari = DV + m_FS_W
		 + m_FS_Screw[0] * m_State.m_Acc[0]
		 + m_FS_Screw[1] * m_State.m_Acc[1]
		 + m_FS_Screw[2] * m_State.m_Acc[2];
}
////////////////////////////////////////////////////////////////////////
SE3& srBallJoint::FS_Transform()
{
	(this->*m_pfnFS_Transfrom)();
	return m_FS_SE3;
}

void srBallJoint::_FS_Transform_Straight()
{
	static SE3 tmp_SE3;
	tmp_SE3.SetOrientation(m_State.m_SO3Pos);
	m_FS_SE3 = m_ChildLinkToJoint * tmp_SE3;
	m_FS_SE3 = m_FS_SE3 / m_ChildLinkToJoint;
}

void srBallJoint::_FS_Transform_Reverse()
{
	static SE3 tmp_SE3;
	tmp_SE3.SetOrientation(m_State.m_SO3Pos);
	m_FS_SE3 = m_ParentLinkToJoint / tmp_SE3;
	m_FS_SE3 = m_FS_SE3 / m_ParentLinkToJoint;
}

se3& srBallJoint::FS_UpdateLocalVelocity()
{
	m_FS_LocalVelocity = (m_FS_Screw[0] * m_State.m_Vel[0]) 
					   + (m_FS_Screw[1] * m_State.m_Vel[1])
					   + (m_FS_Screw[2] * m_State.m_Vel[2]);
	return m_FS_LocalVelocity;
}
se3& srBallJoint::FS_UpdatePosErrorLocalVelocity()
{
	m_FS_LocalVelocity = (m_FS_Screw[0] * m_State.m_PosErrVel[0]) 
  					   + (m_FS_Screw[1] * m_State.m_PosErrVel[1])
					   + (m_FS_Screw[2] * m_State.m_PosErrVel[2]);
	return m_FS_LocalVelocity;
}
void srBallJoint::FS_SetScrew(int i)
{
	if ( i == 0 )
	{
		m_FS_Screw[0] = Ad(m_ChildLinkToJoint, m_Axis[0]);
		m_FS_Screw[1] = Ad(m_ChildLinkToJoint, m_Axis[1]);
		m_FS_Screw[2] = Ad(m_ChildLinkToJoint, m_Axis[2]);
	}
	else if ( i == 1 )
	{
		m_FS_Screw[0] = Ad(m_ChildLinkToJoint, -m_Axis[0]);
		m_FS_Screw[1] = Ad(m_ChildLinkToJoint, -m_Axis[1]);
		m_FS_Screw[2] = Ad(m_ChildLinkToJoint, -m_Axis[2]);
	}
}
void srBallJoint::FS_ResetT()
{
	m_FS_T = 0.0;
}

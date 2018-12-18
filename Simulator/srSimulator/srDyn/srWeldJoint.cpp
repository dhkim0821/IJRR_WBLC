#include "srDyn/srWeldJoint.h"
#include "srDyn/srLink.h"
//#include "../srDyn/srState.h"

srWeldJoint::srWeldJoint()
: srJoint()
{
	m_JointType = WELD;
	GetGeomInfo().SetShape(srGeometryInfo::BOX);
}

srWeldJoint::~srWeldJoint()
{
}

void srWeldJoint::SetDeviceOnOff(bool onoff)
{
}

srState* srWeldJoint::GetStatePtr()
{
	return reinterpret_cast<srState*>(&m_State);
}

void srWeldJoint::Initialize()
{
}

inline void srWeldJoint::FS_UpdateForce(const dse3& /*F*/)
{
	// Do nothing
}

inline void srWeldJoint::FS_UpdateAIS_K(const AInertia& /*AI*/)
{
	// Do nothing
}

inline void srWeldJoint::FS_UpdateAIS_K_P(AInertia& __AI, const AInertia& AI)
{
	__AI = AI;
}

inline void srWeldJoint::FS_UpdateBiasImp(dse3& Cias, const dse3& Bias)
{
	Cias = Bias;
}

inline void srWeldJoint::FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& /*AI*/, const se3& /*V*/)
{
	Cias = Bias;
}

inline void srWeldJoint::FS_UpdateLocalDelVel(se3& jari, const se3& DV)
{
	jari = DV;
}

inline void srWeldJoint::FS_UpdateLocalAcc(se3& jari, const se3& DV)
{
	jari = DV;
}
//////////////////////////////////////////////////////////////////////////
SE3& srWeldJoint::FS_Transform()
{
	return m_FS_SE3;
}
se3& srWeldJoint::FS_UpdateLocalVelocity()
{
	return m_FS_LocalVelocity;
}
se3& srWeldJoint::FS_UpdatePosErrorLocalVelocity()
{
	return m_FS_LocalVelocity;
}
void srWeldJoint::FS_SetScrew(int )
{
}
void srWeldJoint::FS_ResetT()
{
}


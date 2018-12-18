#include "LieGroup/LieGroup.h"
#include "srDyn/srLink.h"
#include "srDyn/srJoint.h"
#include "srDyn/srSystem.h"

srLink::srLink()
{
	m_Restitution = 0.5;
	m_Friction	= 0.4;
	m_Damping	= 0.01;
	m_Inertia	= Inertia(1.0, 0.0017, 0.0017, 0.0017);	//* 10cm-cube with density 1g/cm^3 (water)

	m_ChildJoints.clear();
	m_Collisions.clear();
	m_Sensors.clear();

	m_pSystem = NULL;
	m_IsBaseLink = false;

	m_DynType	= DYNAMIC;

	m_ParentJoint	= NULL;
	m_ParentLink	= NULL;

	m_ChildLinks.clear();

	m_Vel = se3(0.0);
	m_Acc = se3(0.0);
	m_DelVel = se3(0.0);
	m_Vel_PosErr = se3(0.0);
	m_ExtForce = dse3(0.0);
	m_ConstraintImpulse = dse3(0.0);
	m_UserExtForce = dse3(0.0);

}

srLink::~srLink()
{
}

sr_real srLink::GetRestitution()
{
	return m_Restitution;
}

sr_real srLink::GetFriction()
{
	return m_Friction;
}

sr_real srLink::GetDamping()
{
	return m_Damping;
}
void srLink::SetRestitution(sr_real v)
{
	if (v >= 0.0)
		m_Restitution = v;
}
void srLink::SetFriction(sr_real v)
{
	if (v >= 0.0)
		m_Friction = v;
}
void srLink::SetDamping(sr_real v)
{
	if (v >= 0.0)
		m_Damping = v;
}
Inertia& srLink::GetInertiaRef()
{
	return m_Inertia;
}
// void srLink::SetInertia(Inertia v)
// {
// 	m_Inertia = v;
// }

void srLink::SetInertia(Inertia &v)
{
	m_Inertia = v;
}

bool srLink::AddCollision(srCollision* pCollision)
{
	if(m_Collisions.find(pCollision) != -1)
		return false;

	pCollision->m_pLink = this;
	m_Collisions.add_tail(pCollision);
	return true;
}

void srLink::RemoveCollision(srCollision* pCollision)
{
	m_Collisions.remove(pCollision);
}

bool srLink::AddSensor(srSensor* pSensor)
{
	if(m_Sensors.find(pSensor) != -1)
		return false;

	pSensor->m_pLink = this;
	m_Sensors.add_tail(pSensor);
	return true;
}

void srLink::RemoveSensor(srSensor* pSensor)
{
	m_Sensors.remove(pSensor);
}

void srLink::UpdateInertia(sr_real density)
{
	switch (m_GeomInfo.m_Type)
	{
	case srGeometryInfo::BOX:
		m_Inertia = BoxInertia(density, 0.5*m_GeomInfo.GetDimension());
		break;
	case srGeometryInfo::SPHERE:
		m_Inertia = SphereInertia(density, 0.5*m_GeomInfo.GetDimension()[0]);
		break;
	case srGeometryInfo::CAPSULE:
		m_Inertia = CapsuleInertia(density, 0.5*m_GeomInfo.GetDimension()[0], m_GeomInfo.GetDimension()[1]);
		break;
	case srGeometryInfo::CYLINDER:
		m_Inertia = CylinderInertia(density, 0.5*m_GeomInfo.GetDimension()[0], m_GeomInfo.GetDimension()[1]);
		break;
	default:
		break;
	}
}

const se3& srLink::GetVel()
{
	return m_Vel;
}

const se3& srLink::GetAcc()
{
	return m_Acc;
}

const dse3& srLink::GetExtForce()
{
	return m_ExtForce;
}

sr_real srLink::GetMass()
{
	return m_Inertia.GetMass();
}

Vec3 srLink::GetOffset()
{
	return m_Inertia.GetOffset();
}

void srLink::SetOffset(Vec3 ofs)
{
	m_Inertia.SetOffset(ofs);
}

Vec3 srLink::GetMassCenter()
{
	return m_Frame.GetPosition() + m_Frame.GetOrientation() * GetOffset();
}

Vec3 srLink::GetLinearVel(Vec3 p)
{
	return Rotate(m_Frame, MinusLinearAd(p, m_Vel));
}

Vec3 srLink::GetLinearAcc(Vec3 p)
{
	return Rotate(m_Frame, MinusLinearAd(p, m_Acc)-ad(MinusLinearAd(p, m_Vel), m_Vel));
}

void srLink::AddUserExternalForce(dse3 v)
{
	m_UserExtForce = v;
}

void srLink::ResetUserExternalForce()
{
	m_UserExtForce = dse3(0.0);
}

void srLink::SetDynType()
{
	// DYNTYPE
	if ( m_IsBaseLink && (m_pSystem->m_BaseLinkType == srSystem::FIXED) )
	{
		m_Vel = 0.0;
		m_Acc = 0.0;
		m_DelVel = 0.0;

		m_DynType = STATIC;
	}
	else if ( m_IsBaseLink && (m_pSystem->m_BaseLinkType == srSystem::KINEMATIC) )
	{
		m_DynType = KINEMATIC;
	}
	else
	{
		m_DynType = DYNAMIC;
	}
}

SE3& srLink::FS_Transform()
{
	m_MexpSq = m_M * m_ParentJoint->FS_Transform();
	return m_MexpSq;
}

SE3& srLink::FS_GetSE3()
{
	return m_MexpSq;
}

void srLink::FS_UpdateAIS_K(AInertia &AIjari)
{
	m_ParentJoint->FS_UpdateAIS_K_P(AIjari, m_AInertia);
}

void srLink::FS_UpdateAIS_K(void)
{
	m_ParentJoint->FS_UpdateAIS_K(m_AInertia);
}

void srLink::FS_UpdateBiasImp(dse3 &jari)
{
	m_ParentJoint->FS_UpdateBiasImp(jari, m_Bias);
}

void srLink::FS_UpdateBiasforce(dse3 &jari)
{
	m_ParentJoint->FS_UpdateBiasforce(jari, m_Bias, m_AInertia, m_Vel);
}

void srLink::FS_UpdateDelVel(se3 &DV)
{
	m_ParentJoint->FS_UpdateLocalDelVel(m_DelVel,DV);
}

void srLink::FS_UpdateAcc(se3 &DV)
{
	m_ParentJoint->FS_UpdateLocalAcc(m_Acc, DV);
}

void srLink::FS_UpdateForce_Link()
{
	m_ParentJoint->m_FS_Force = m_AInertia*m_Acc + m_Bias;
	m_ParentJoint->FS_UpdateForce(m_ParentJoint->m_FS_Force);
}

void srLink::FS_UpdateImpulse_Link(sr_real & _fps)
{
	m_ParentJoint->m_FS_Force += (_fps * (m_AInertia * m_DelVel + m_Bias));
}

void srLink::ClearHistory()
{
	m_History.clear();
}

void srLink::BackupInitState()
{
	m_InitFrame = m_Frame;
	m_InitVel	= m_Vel;
}

void srLink::RestoreInitState()
{
	m_Frame	= m_InitFrame;
	m_Vel	= m_InitVel;
}

void srLink::PushState()
{
	m_History.add_tail(m_Frame);
}

void srLink::PopState(int index)
{
	if(m_History.get_size() == 0)
		return;

	if(index >= m_History.get_size() || index < 0)
		index = m_History.get_size() - 1;

	m_Frame = m_History[index];
}

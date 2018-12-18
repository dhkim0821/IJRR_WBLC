#include "srDyn/srJoint.h"
#include "srDyn/srLink.h"

srJoint::srJoint()
{
	m_ActType = PASSIVE;
	m_ParentLink = NULL;
	m_ChildLink = NULL;
	m_ParentLinkToJoint = SE3();
	m_ChildLinkToJoint = SE3();
	m_PriorityIndex = 0;
	m_IsDeviceOn = true;
	m_FS_SE3 = SE3();
	m_FS_LocalVelocity = se3(0.0);
	m_FS_Force = dse3(0.0);
	m_FS_Force_SE3 = SE3();
}

srJoint::~srJoint()
{
}

srJoint::ACTTYPE srJoint::GetActType()
{
	return m_ActType;
}

void srJoint::SetActType(ACTTYPE v)
{
	m_ActType = v;
}

srJoint::JOINTTYPE srJoint::GetType()
{
	return m_JointType;
}

void srJoint::SetParentLink(srLink* pLink)
{
	m_ParentLink = pLink;
	m_ParentLink->m_ChildJoints.add_tail(this);
}

void srJoint::SetChildLink(srLink* pLink)
{
	m_ChildLink = pLink;
}

SE3& srJoint::GetParentLinkFrame()
{
	return m_ParentLinkToJoint;
}

SE3& srJoint::GetChildLinkFrame()
{
	return m_ChildLinkToJoint;
}

void srJoint::SetParentLinkFrame(SE3 v)
{
	m_ParentLinkToJoint = v;
}

void srJoint::SetChildLinkFrame(SE3 v)
{
	m_ChildLinkToJoint = v;
}

void srJoint::SetPriorityIndex(int idx)
{
	if (idx == 0)
		m_PriorityIndex = 0;
	else
		m_PriorityIndex = 1;
}

bool srJoint::IsDeviceOn()
{
	return m_IsDeviceOn;
}

//////////////////////////////////////////////////////////////////////////
void srJoint::UpdateFrame()
{
	(this->*m_pfn_updateframe)();
}

void srJoint::Set_UpdateFrame_Ftn()
{
	if (m_ParentLink == NULL && m_ChildLink == NULL)
	{
		m_pfn_updateframe = &srJoint::_updateframe_DoNothing;
	}
	else if (m_ChildLink != NULL && m_PriorityIndex == 1)
	{
		m_pfn_updateframe = &srJoint::_updateframe_Secondary;
	}
	else //if (m_ParentLink != NULL && m_PriorityIndex == 0)
	{
		m_pfn_updateframe = &srJoint::_updateframe_Primary;
	}
}

void srJoint::_updateframe_Primary()
{
	m_Frame = m_ParentLink->GetFrame() * m_ParentLinkToJoint;
}

void srJoint::_updateframe_Secondary()
{
	m_Frame = m_ChildLink->GetFrame() * m_ChildLinkToJoint;
}

void srJoint::_updateframe_DoNothing()
{
}

dse3 srJoint::GetFT()
{
	return 	dAd(m_FS_Force_SE3, m_FS_Force);
}


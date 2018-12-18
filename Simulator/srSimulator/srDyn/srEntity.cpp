#include <memory.h>
#include "srDyn/srEntity.h"


srEntity::srEntity()
{
	m_Frame = SE3();

	m_Position = Vec3();
	m_EulerAngle = InvVec3();
	m_Orientation = SO3();
}

srEntity::~srEntity()
{
}

SE3& srEntity::GetFrame()
{
	return m_Frame;
}

Vec3& srEntity::GetPosition()
{
	m_Position = m_Frame.GetPosition();
	return m_Position;
}

SO3& srEntity::GetOrientation()
{
	m_Orientation = m_Frame.GetOrientation();
	return m_Orientation;
}

InvVec3& srEntity::GetEulerAngle()
{
	m_EulerAngle = iEulerZYX(m_Frame);
	return m_EulerAngle;
}

void srEntity::SetFrame(SE3 v)
{
	m_Frame = v;
}

//void srEntity::SetFrame(SE3& v)
//{
//	m_Frame = v;
//}

void srEntity::SetPosition(Vec3 v)
{
	m_Frame.SetPosition(v);
}

void srEntity::SetOrientation(SO3 v)
{
	m_Frame.SetOrientation(v);
}

void srEntity::SetEulerAngle(InvVec3 v)
{
	m_Frame.SetOrientation(EulerZYX(v));
}

srGeometryInfo& srEntity::GetGeomInfo()
{
	return m_GeomInfo;
}

void srEntity::SetGeomInfo(srGeometryInfo& v)
{
	memcpy(&m_GeomInfo, &v, sizeof(srGeometryInfo));
}


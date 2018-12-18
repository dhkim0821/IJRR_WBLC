#include "srDyn/srSensor.h"

srSensor::srSensor()
{
	m_pLink = NULL;

	m_LocalFrame = SE3();
}
srSensor::~srSensor()
{
}

SE3& srSensor::GetLocalFrame()
{
	return m_LocalFrame;
}

void srSensor::SetLocalFrame(SE3 T)
{
	m_LocalFrame = T;
}

void srSensor::SetLocalFrame(SE3& T)
{
	m_LocalFrame = T;
}

void srSensor::UpdateFrame()
{
	m_Frame = m_pLink->m_Frame * m_LocalFrame;
}


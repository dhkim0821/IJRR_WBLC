#include "srDyn/srCollision.h"
#include "srDyn/srIRSensor.h"

#include <stdio.h>

srIRSensor::srIRSensor()
: srSensor()
{
	m_SensorType = IRSENSOR;
	m_MaxRange = 1.0;
	m_MinRange = 0.0;
	m_DetectedValue = 1.0;
	m_Objects.clear();
}

sr_real& srIRSensor::GetDetectedValue()
{
	return m_DetectedValue;
}

sr_real srIRSensor::GetMaxRange()
{
	return m_MaxRange;
}

sr_real srIRSensor::GetMinRange()
{
	return m_MinRange;
}

void srIRSensor::SetMaxRange(sr_real max)
{
	if(max > m_MinRange) {
		m_MaxRange = max;
	}
	else{
		printf("IR sensor maximum range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

void srIRSensor::SetMinRange(sr_real min)
{
	if(m_MaxRange > min && min >= 0.0) {
		m_MinRange = min;
	}
	else{
		printf("IR sensor minimum range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

void srIRSensor::SetRange(sr_real max, sr_real min)
{
	if(max > min && min >= 0.0) {
		m_MaxRange = max;
		m_MinRange = min;
	}
	else {
		printf("IR sensor range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

void srIRSensor::Initialize()
{
	m_DetectedValue = m_MaxRange;
}

void srIRSensor::ResetSensor()
{
	m_DetectedValue = m_MaxRange;
}

void srIRSensor::AddObject(srCollision* object)
{
	m_Objects.add_tail(object);
}

void srIRSensor::RemoveObject(srCollision* object)
{
	m_Objects.remove(object);
}

void srIRSensor::ClearObject()
{
	m_Objects.clear();
}

void srIRSensor::Detect()
{
	//* position of the starting point of IR ray in global coordinate.
	Vec3 position(&GetFrame()[9]);
	//* x-axis unit vector of body frame in global coordinate.
	Vec3 dir(&GetFrame()[0]);
	//* directional vector in global coordinate from the starting point of IR ray 
	//* to the center of collision object.
	Vec3 cc_n;
	double norm_cc, dist;
	int iCount;

	srCollision* pCol;
	m_DetectedValue = m_MaxRange;

	iCount = m_Objects.get_size();	
	for(int i = 0 ; i < iCount ; ++i)
	{
		pCol = m_Objects[i];

		//* Rough Check
		cc_n = pCol->GetFrame().GetPosition() - position;
		norm_cc = cc_n.Normalize();

		if(norm_cc > (Inner(m_MaxRange * dir, cc_n) + pCol->GetBoundingRadius())) continue;

		//* Detect
		if(pCol->RayDetection(position, dir, m_MaxRange, dist))
		{
			m_DetectedValue = min(m_DetectedValue, dist);

			//* If sensor detects distance below m_MinRange value,
			//* Initialize its value and return;
			if(m_DetectedValue < m_MinRange)
			{
				m_DetectedValue = SR_SENSOR_MSG_NA;
				return;
			}
		}
	}
}

void srIRSensor::Draw()
{
}

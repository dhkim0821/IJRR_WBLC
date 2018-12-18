#include "srDyn/srCollision.h"
#include "srDyn/srRangeFinder.h"
#include <stdio.h>

srRangeFinder::srRangeFinder()
: srSensor()
{
	m_SensorType = RANGEFINDER;

	m_MaxRange = 1.0;
	m_MinRange = 0.0;
	m_Spread = 180;
	m_Resolution = 1.0;

	m_NumSpots = (int)((sr_real)m_Spread / m_Resolution) + 1;

	m_SpreadRad = DEG2RAD((sr_real)m_Spread * 0.5);
	m_ResRad = DEG2RAD(m_Resolution);

	for(int i = 0 ; i < m_NumSpots ; ++i)
		m_DetectedValue[i] = 1.0;

	m_Objects.clear();
}
sr_real srRangeFinder::GetMaxRange()
{
	return m_MaxRange;
}

sr_real srRangeFinder::GetMinRange()
{
	return m_MinRange;
}

void srRangeFinder::SetMaxRange(sr_real max)
{
	if(max > m_MinRange) {
		m_MaxRange = max;
	}
	else{
		printf("IR sensor maximum range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

void srRangeFinder::SetMinRange(sr_real min)
{
	if(m_MaxRange > min && min >= 0.0) {
		m_MinRange = min;
	}
	else{
		printf("IR sensor minimum range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

void srRangeFinder::SetRange(sr_real max, sr_real min)
{
	if(max > min && min >= 0.0) {
		m_MaxRange = max;
		m_MinRange = min;
	}
	else {
		printf("IR sensor range setting incorrect: Max range > Min range => 0.0.\n");
	}
}

int srRangeFinder::GetSpread()
{
	return m_Spread;
}

void srRangeFinder::SetSpread(int s)
{
	if( 0 < s && s < 360)
	{
		m_Spread = s;
		m_NumSpots = (int)((sr_real)m_Spread / m_Resolution) + 1;
		m_SpreadRad = DEG2RAD((sr_real)m_Spread * 0.5);
	}
}
sr_real srRangeFinder::GetResolution()
{
	return m_Resolution;
}
void srRangeFinder::SetResolution(sr_real r)
{
	if(0.0 < r && r < m_Spread)
	{
		m_Resolution = r;
		m_NumSpots = (int)((sr_real)m_Spread / m_Resolution) + 1;
		m_ResRad = DEG2RAD(m_Resolution);
	}
}
sr_real* srRangeFinder::GetDetectedValue()
{
	return m_DetectedValue;
}
void srRangeFinder::Initialize()
{
	m_NumSpots = (int)((sr_real)m_Spread / m_Resolution) + 1;

	m_SpreadRad = DEG2RAD((sr_real)m_Spread * 0.5);
	m_ResRad = DEG2RAD(m_Resolution);

	for(int i = 0 ; i < m_NumSpots ; ++i)
		m_DetectedValue[i] = m_MaxRange;
}

void srRangeFinder::ResetSensor()
{
	for(int i = 0 ; i < m_NumSpots ; ++i)
		m_DetectedValue[i] = m_MaxRange;
}

void srRangeFinder::AddObject(srCollision* object)
{
	m_Objects.add_tail(object);
}

void srRangeFinder::RemoveObject(srCollision* object)
{
	m_Objects.remove(object);
}

void srRangeFinder::ClearObject()
{
	m_Objects.clear();
}

void srRangeFinder::Detect()
{
	//* position of the starting point of IR ray in global coordinate.
	Vec3 position(&GetFrame()[9]);
	//* IR ray direction in global coordinate
	Vec3 dir;
	//* IR ray direction in local coordinate
	Vec3 dir_local;
	//* directional vector in global coordinate from the starting point of IR ray to the center of collision object.
	Vec3 cc_n;
	//* IR ray angle in radian. It grows from - m_Spread/2 to m_Spread/2
	sr_real fTheta;
	int iCount;

	ResetSensor();

	iCount = m_Objects.get_size();	
	for(int j = 0 ; j < m_NumSpots ; ++j)
	{
		//* Get IR ray direction
		fTheta = m_ResRad * j - m_SpreadRad;

		dir_local = Vec3(cos(fTheta), sin(fTheta), 0.0);
		dir = GetFrame().GetOrientation() * dir_local;

		_DetectOneSpot(j, dir, position);
	}
}

void srRangeFinder::_DetectOneSpot(int &idx, Vec3 &direction, Vec3 &position)
{
	//* Position vector from the starting point of sensor to the origin of collision object.
	Vec3 cc_n;
	//* Norm of cc_n;
	sr_real norm_cc;
	//* Detected distance.
	sr_real dist;

	for(int i = 0 ; i < m_Objects.get_size() ; ++i)
	{
		srCollision* pCol = m_Objects[i];

		//* Rough Check
		cc_n = pCol->GetFrame().GetPosition() - position;
		norm_cc = cc_n.Normalize();

		if(norm_cc > (Inner(m_MaxRange * direction, cc_n) + pCol->GetBoundingRadius())) continue;

		//* Detect
		if(pCol->RayDetection(position, direction, m_MaxRange, dist))
		{
			m_DetectedValue[idx] = min(m_DetectedValue[idx], dist);

			//* If sensor detects distance below m_MinRange value,
			//* Initialize its value and return;
			if(m_DetectedValue[idx] < m_MinRange)
			{
				m_DetectedValue[idx] = SR_SENSOR_MSG_NA;
				return;
			}
		}
	}
}

int srRangeFinder::GetNumSpots()
{
	return m_NumSpots;
}

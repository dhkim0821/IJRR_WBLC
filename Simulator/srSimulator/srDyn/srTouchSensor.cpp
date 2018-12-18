#include "srDyn/srCollision.h"
#include "srDyn/srTouchSensor.h"

srTouchSensor::srTouchSensor()
: srSensor()
{
	m_SensorType = TOUCHSENSOR;

	m_Range = 0.05;
	m_DetectedValue = false;
	m_Objects.clear();

	GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	GetGeomInfo().SetDimension(0.1f, 0.0f, 0.0f);
}

bool srTouchSensor::GetDetectedValue()
{
	return m_DetectedValue;
}

sr_real srTouchSensor::GetRange()
{
	return m_Range;
}

void srTouchSensor::SetRange(sr_real R)
{
	if(R > 0.0)
	{
		m_Range = R;

		GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
		GetGeomInfo().SetDimension((float)(m_Range*2), 0.0f, 0.0f);
	}
}

void srTouchSensor::Initialize()
{
	m_DetectedValue = false;
}

void srTouchSensor::ResetSensor()
{
	m_DetectedValue = false;
}

void srTouchSensor::AddObject(srCollision* object)
{
	m_Objects.add_tail(object);
}

void srTouchSensor::RemoveObject(srCollision* object)
{
	m_Objects.remove(object);
}

void srTouchSensor::ClearObject()
{
	m_Objects.clear();
}

void srTouchSensor::Detect()
{
	//* position of the starting point of IR ray in global coordinate.
	Vec3 position(GetFrame().GetPosition());

	//* dummy for TouchDetection function argument
	Vec3 point;
	Vec3 normal;
	sr_real penetration;
	sr_real tmp;
	int iCount;


	ResetSensor();

	iCount = m_Objects.get_size();	
	for(int i = 0 ; i < iCount ; ++i)
	{
		srCollision* pCol = m_Objects[i];

		//* Check bounding sphere
		tmp = pCol->GetBoundingRadius() + m_Range;
		tmp *= tmp;
		if( SquareSum(position - pCol->GetFrame().GetPosition()) > tmp ) continue;

		//* Detect
		if(pCol->TouchDetection(position, m_Range, point, normal, penetration))
		{
			m_DetectedValue = true;
			// notify the fact that this sensor sensed something to registered function
			NotifyDetected();
			return;
		}
	}
}

void srTouchSensor::SetDetectedCallback(void (*pfn)(void*), void* pVoid)
{
	m_pfn_DetectedCB = pfn;
	m_pvDetectedData = pVoid;
}

void srTouchSensor::NotifyDetected()
{
	(*m_pfn_DetectedCB)(m_pvDetectedData);
}

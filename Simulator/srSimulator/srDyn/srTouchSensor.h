#ifndef SRLIB_TOUCH_SENSOR
#define SRLIB_TOUCH_SENSOR

#include "srDyn/srSensor.h"
#include "LieGroup/_array.h"

/*!
	\class srTouchSensor
	\brief Class represents touch sensor.
*/
class srTouchSensor : public srSensor
{
public:
	/*!
		Bounding radius of touch sensor. This should be specified by user.
		This must be greater than zero.
		Default is 0.05.
	*/
	sr_real	m_Range;
	/*!
		Get bounding radius of touch sensor.
	*/
	sr_real	GetRange();
	/*!
		Set bounding radius of touch sensor.
	*/
	void	SetRange(sr_real );
public:
	/*!
	Constructor.
	*/
	srTouchSensor();
	/*!
		List of collision entities that sensor can detect.
	*/
	_array<srCollision*>	m_Objects;
	/*!
		Detected value of sensor.
	*/
	bool	m_DetectedValue;
	/*!
		Reset detected value of sensor.
	*/
	void	ResetSensor();
	/*!
		Get Detected value of sensor.
	*/
	bool	GetDetectedValue();

	/*!
		Add collision entity to detection list.
	*/
	void	AddObject(srCollision* );
	/*!
		Remove collision entity to detection list.
	*/
	void	RemoveObject(srCollision* );
	/*!
		Reset detection list.
	*/
	void	ClearObject();

	/*!
		Initialize sensor. This function is called once before simulation.
	*/
	void	Initialize();
	/*!
		Sensor operate.
	*/
	void	Detect();

	/*!
		Set detected call back function that will be called when this sensor detect something.
	*/
	void	SetDetectedCallback(void (*pfn)(void*), void* pVoid);
	
	/*!
		Notify the fact that this sensor sensed something to registered function.
	*/
	void	NotifyDetected();

	/*!
		Function pointer.
	*/
	void	(*m_pfn_DetectedCB)(void* pVoid);

	/*!
		Data of function pointer.
	*/
	void*	m_pvDetectedData;
};

#endif


#ifndef SRLIB_SENSOR
#define	SRLIB_SENSOR

#include "LieGroup/LieGroup.h"
#include "srDyn/srEntity.h"
#include "srDyn/srLink.h"

/*!
	\class srSensor
	\brief Class represents sensor object.

	srSensor is an abstract class.
*/
class srSensor : public srEntity
{
public:
	/*!
		Relative frame from link entity to sensor entity.
		This should be specified by user. Default is identity
	*/
	SE3		m_LocalFrame;
public:
	/*!
		Get local frame.
	*/
	SE3&	GetLocalFrame();
	/*!
		Set local frame.
	*/
	void	SetLocalFrame(SE3 );
	/*!
		Set local frame.
	*/
	void	SetLocalFrame(SE3& );

//////////////////////////////////////////////////////////////////////////
public:
	/*!
		Constructor.
	*/
	srSensor();
	/*!
		Destructor.
	*/
	~srSensor();
	/*!
		Parent link entity to which sensor entity belongs.
	*/
	srLink* m_pLink;
	enum SENSORTYPE { IRSENSOR, RANGEFINDER, TOUCHSENSOR };
	/*!
		Sensor type. 
		IRSENSOR, RANGEFINDER, TOUCHSENSOR.
	*/
	SENSORTYPE m_SensorType;
	/*!
		Update global frame referring to parent link frame.
	*/
	void	UpdateFrame();
	/*!
		Initialize sensor. This function is called once before simulation.
	*/
	virtual void Initialize() = 0;
	/*!
		Sensor operate.
	*/
	virtual void Detect() = 0;
};

#endif

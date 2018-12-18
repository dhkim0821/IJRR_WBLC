#ifndef	SRLIB_ENTITY
#define SRLIB_ENTITY

#include "LieGroup/LieGroup.h"
#include "srDyn/srObject.h"
#include "srDyn/srGeometryInfo.h"

/*!
	\class srEntity
	\brief Base class of Link, Joint, Collision, Sensor elements.

	srEntity is a class for design elements which consist of a robot mechanism.
	srEntity is an abstract class.
	Derived classes from srEntity can have its own position and orientation in 3D space
	and can be shown visually on screen because this has srGeometryInfo class.
*/
class srEntity : public srObject
{
public:
	/*!
		Constructor.
	*/
			srEntity();
	/*!
		Destructor.
	*/
	virtual ~srEntity();

	/*!
		SE(3) variable which represents position and orientation in 3D space.
	*/
	SE3			m_Frame;
	/*!
		Geometry information of entity.
	*/
	srGeometryInfo	m_GeomInfo;
	/*!
		Get SE(3) frame.
	*/
	SE3&		GetFrame();
	/*!
		Get position in Vec3 form.
	*/
	Vec3&		GetPosition();
	/*!
		Get orientation in EulerZYX angle form.
	*/
	InvVec3&		GetEulerAngle();
	/*!
		Get orientation in SO(3) form.
	*/
	SO3&			GetOrientation();
	/*!
		Set SE(3) frame.
	*/
	void		SetFrame(SE3 );
	/*!
		Set SE(3) frame.
	*/
//	void		SetFrame(SE3& );
	/*!
		Set position.
	*/
	void		SetPosition(Vec3 );
	/*!
		Set orientation.
	*/
	void		SetEulerAngle(InvVec3 );
	/*!
		Set orientation.
	*/
	void		SetOrientation(SO3 );
	/*!
		Get geometry information.
	*/
	srGeometryInfo&	GetGeomInfo();
	/*!
		Set geometry information.
	*/
	void		SetGeomInfo(srGeometryInfo& );
	/*!
		User can make own draw function.
	*/
	virtual void	Draw() {};
protected:
	/*!
		Position of entity in Vec3 form.
	*/
	Vec3		m_Position;		// Position
	/*!
		Orientation of entity in SO(3) form.
	*/
	SO3			m_Orientation;
	/*!
		Orientation of entity in EulerZYX form(InvVec3).
	*/
	InvVec3		m_EulerAngle;
};

#endif

#ifndef	SRLIB_COLLISION
#define SRLIB_COLLISION

#include "srDyn/srEntity.h"
#include "srDyn/srLink.h"
#include "LieGroup/LieGroup.h"

/*!
	\class srCollision
	\brief Class represents collision object.
*/
class srCollision : public srEntity
{
public:
	/*!
		Relative frame from link entity to collision entity.
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
	srCollision();
	/*!
		Destructor.
	*/
	~srCollision();
	/*!
		Parent link entity to which collision entity belongs.
	*/
	srLink* m_pLink;
	/*!
		Bounding radius for collision pair detection and sensor detection.
		UpdateBoundingRadius function will be called automatically to compute this variable.
	*/
	sr_real	m_BoundingRadius;
public:
	/*!
		Returns collision shape type
	*/
	srGeometryInfo::SHAPETYPE GetCollisionShape();
	/*!
		Returns collision shape dimension
	*/
	Vec3&	GetDimension();
	/*!
		Update global frame referring to parent link frame.
	*/
	void	UpdateFrame();
	/*!
		Compute and update bounding radius referring to geometry information.
	*/
	void	UpdateBoundingRadius();
	/*!
		Get bounding radius.
	*/
	sr_real&	GetBoundingRadius();
	/*!
		Set RayDetection function referring to geometry information.
	*/
	void	UpdateRayDetectionFtn();
	/*!
		Function for IRSensor to detection whether collision entity intervenes or not.
	*/
	bool	RayDetection(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	/*!
		Function pointer for RayDetection function.
	*/
	bool	(srCollision::*m_pfn_RayDetect)(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToBox		(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToSphere	(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToCylinder	(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToCapsule	(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToPlane		(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);
	bool	_RayToUser		(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist);

	/*!
		Set TouchDetection function referring to geometry information.
	*/	
	void	UpdateTouchDetectionFtn();
	/*!
		Function for TouchSensor to detection whether collision entity intervenes or not.
	*/	
	bool	TouchDetection(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	/*!
		Function pointer for TouchDetection function.
	*/
	bool	(srCollision::*m_pfn_TouchDetect)(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToBox		(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToSphere	(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToCylinder(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToCapsule	(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToPlane	(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	bool	_TouchToUser	(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration);
	/*!
		Set SonarDetection function referring to geometry information.
	*/	
	void	UpdateSonarDetectionFtn();
	/*!
		Function for SonarDetection to detection whether collision entity intervenes or not.
	*/	
	bool	SonarDetection(SE3& T0, sr_real & range , sr_real & angle, bool & res);
	/*!
		Function pointer for SonarDetection function.
	*/
	bool	(srCollision::*m_pfn_SonarDetect)(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToBox		(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToSphere	(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToCylinder(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToCapsule	(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToPlane	(SE3& T0, sr_real & range, sr_real & angle, bool & res);
	bool	_SonarToUser	(SE3& T0, sr_real & range, sr_real & angle, bool & res);
};

#endif

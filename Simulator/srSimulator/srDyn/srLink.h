#ifndef	SRLIB_LINK
#define SRLIB_LINK

#include "LieGroup/_array.h"
#include "srDyn/srEntity.h"

class srJoint;
class srCollision;
class srSensor;
class srSystem;


/*!
	\class srLink
	\brief Class represents link object.
*/
class srLink : public srEntity
{

public:
	/*!
		Restitution coefficient of link entity. This should be specified by user.
		Default is 0.5.
	*/
	sr_real	m_Restitution;
	/*!
		Friction coefficient of link entity. This should be specified by user.
		Default is 0.4;
	*/
	sr_real	m_Friction;
	/*!
		Damping coefficient of link entity. This should be specified by user.
		Default is 0.01;
	*/
	sr_real	m_Damping;
	/*!
		Generalized inertia of link entity. This should be specified by user.
		Default is for 10cm-cube with density 1g/cm^3 (water).
	*/
	Inertia		m_Inertia;
	/*!
		List of collision entities. This should be specified by user.
		Function AddCollision do this job.
	*/
	_array<srCollision*>	m_Collisions;
	/*!
		List of sensor entities. This should be specified by user.
		Function AddSensor do this job.
	*/
	_array<srSensor*>		m_Sensors;
	/*!
		Get restitution coefficient.
	*/
	sr_real	GetRestitution();
	/*!
		Get friction coefficient.
	*/
	sr_real	GetFriction();
	/*!
		Get damping coefficient.
	*/
	sr_real	GetDamping();
	/*!
		Set restitution coefficient.
	*/
	void	SetRestitution(sr_real );
	/*!
		Set friction coefficient.
	*/
	void	SetFriction(sr_real );
	/*!
		Set damping coefficient.
	*/
	void	SetDamping(sr_real );
	/*!
		Get generalized inertia.
	*/
	Inertia&	GetInertiaRef();
	/*!
		Set generalized inertia.
	*/
//	void		SetInertia(Inertia v); 
	/*!
		Set generalized inertia.

	*/

	void		SetInertia(Inertia& v);
	/*!
		Add collision entity to list of collision entities.
	*/
	bool		AddCollision(srCollision* );
	/*!
		Remove collision entity from list of collision entities.
	*/
	void		RemoveCollision(srCollision* );
	/*!
		Add sensor entity to list of sensor entities.
	*/
	bool		AddSensor(srSensor* );
	/*!
		Remove collision entity from list of sensor entities.
	*/
	void		RemoveSensor(srSensor* );
	/*!
		Update inertia referring to geometry information of entity.
		Default is of water, 1000kgf/m^3 (1gf/cm^3).
	*/
	void		UpdateInertia(sr_real density = 1000.0);
public:
	/*!
		Constructor
	*/
	srLink();
	/*!
		Destructor
	*/
	virtual	~srLink();
	/*!
		Parent system to which link entity belongs.
	*/
	srSystem*				m_pSystem;
	/*!
		Boolean value for whether this is base link or not.
	*/
	bool					m_IsBaseLink;
	enum DYNTYPE { DYNAMIC, STATIC, KINEMATIC };
	/*!
		Link type for dynamics response. Used to determine impulse test function.
		DYNAMIC : link can move. link can respond to external force.
		STATIC : link is fixed. link can not respond to external force.
		KINEMATIC : link can move. link can not respond to external force.
	*/
	DYNTYPE		m_DynType;
	/*!
		Parent joint.
	*/
	srJoint*		 m_ParentJoint;
	/*!
		Parent link,
	*/
	srLink*			 m_ParentLink;
	/*!
		List of child joints.
	*/
	_array<srJoint*>		m_ChildJoints;
	/*!
		List of child links
	*/
	_array<srLink*> m_ChildLinks;
	/*!
		Generalized body velocity.
	*/
	se3			m_Vel;
	/*!
		Generalized body acceleration.
	*/
	se3			m_Acc;
	/*!
		Difference of body velocity. Used in impulse dynamics routines.
	*/
	se3			m_DelVel;
	/*!
		Velocity for manifold error correction. Used in impulse dynamics.
	*/
	se3			m_Vel_PosErr;
	/*!
		Total External force exerted on the link.
		All the external forces - user external force, gravity force and constraint force(e.g contact force) - are summed up to this value.	*/
	dse3		m_ExtForce;
	/*!
		External force that user can exert on the link.
		If user want to exert external force on the link, user has to specify this value at every time step in control loop.
		At the end of every time step this value is set to zero.
		Valid only in dynamics mode.		
	*/
	dse3		m_UserExtForce;
	/*!
		Constraint impulse force. Used in impulse dynamics routines.
	*/
	dse3		m_ConstraintImpulse;
	/*!
		Articulated inertia. Used in forward dynamics routines.
	*/
	AInertia	m_AInertia;
	/*!
		Bias force. Used in forward dynamics routines.
	*/
	dse3		m_Bias;
	/*!
		Initial frame from parent link to this link. Used in forward kinematics routines.
	*/
	SE3			m_M;
	/*!
		Updated frame from parent link to this link referring to parent joint angle. Used in forward kinematics routines.
	*/
	SE3			m_MexpSq;

	/*!
		History of frame. This variable is for replay of simulation.
	*/
	_array<SE3>	m_History;
	/*!
		Initial frame.
	*/
	SE3		m_InitFrame;
	/*!
		Initial body velocity.
	*/
	se3		m_InitVel;

public:
	/*!
		Get body velocity;
	*/
	const se3&	GetVel();
	/*!
		Get body acceleration.
	*/
	const se3&	GetAcc();
	/*!
		Get total external force.
	*/
	const dse3&	GetExtForce();
	/*!
		Get mass.
	 */
	sr_real	GetMass();
	/*!
		Get offset.
		Offset of mass center from the origin of local coordinates.
	 */
	Vec3	GetOffset();
	/*!
		Set offset. in local coordinates
	 */
	void	SetOffset(Vec3 );
	/*!
		Get mass center w.r.t. the global coordinates.
	 */
	Vec3	GetMassCenter();
	/*!
		Get linear velocity of a point `p' on a link.
		If p is a zero vector, this returns the linear velocity of the center of a link.
	 */
	Vec3	GetLinearVel(Vec3 p);
	/*!
		Get linear acceleration of a point `p' on a link.
		If p is a zero vector, this returns the linear acceleration of the center of a link.
	*/
	Vec3	GetLinearAcc(Vec3 p);
	/*!
		Add user-external force. At the end of every time step this value is set to zero.
		If user want to exert external force on the link, user has to specify this value at every time step in control loop.
		This value is added to total external force(m_Extforce) with the other forces-gravity force and constraint force(e.g contact force).
		User can observe total external force value using function GetExtForce in the control loop at every time step.
	*/
	void		AddUserExternalForce(dse3 v);
	
	/*!
		Reset the force defined by user to zero.
	 */
	void		ResetUserExternalForce();

	/*!
		Determine link type for dynamics response.
	*/
	void		SetDynType();
	/*!
		Forward kinematics related routines
	*/
	SE3&		FS_Transform(void);
	SE3&		FS_GetSE3(void);
	/*!
		Forward dynamics related routines
	*/
	virtual void		FS_UpdateAIS_K(AInertia & AIjari);
	virtual void		FS_UpdateAIS_K(void);
	virtual void		FS_UpdateBiasImp(dse3 & jari);
	virtual void		FS_UpdateBiasforce(dse3 & jari);
	virtual void		FS_UpdateDelVel(se3& DV);
	virtual void		FS_UpdateAcc(se3& DV);
	virtual void		FS_UpdateForce_Link();
	virtual void		FS_UpdateImpulse_Link(sr_real & _fps);


	//SE3&	GetInitFrame();
	//se3&	GetInitVel();
	//se3&	GetInitAcc();
	//void	SetInitFrame(SE3 );
	//void	SetInitVel(se3 );
	//void	SetInitAcc(se3 );
	void	ClearHistory();
	void	BackupInitState();
	void	RestoreInitState();
	void	PushState();
	void	PopState(int );

protected:
	int dummy[10];
};
#endif


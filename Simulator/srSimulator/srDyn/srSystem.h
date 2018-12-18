#ifndef	SRLIB_SYSTEM
#define SRLIB_SYSTEM

#include "LieGroup/_array.h"
#include "srDyn/srObject.h"

#include "srDyn/srLink.h"
#include "srDyn/srState.h"
#include "srDyn/srJoint.h"
#include "srDyn/srCollision.h"
#include "srDyn/srSensor.h"


/*!
	\class srSystem
	\brief Class represents system -- a robot.
*/
class srSystem : public srObject
{
public:
	/*!
		Constructor.
	*/
	srSystem();

	/*!
		Space pointer.
	 */
//	srSpace*  m_Space;

	/*!
		Base link of system. This should be specified by user.
		There must be base link to build system.
	*/
	srLink *	 m_BaseLink;
	enum	BASELINKTYPE	{ DYNAMIC, FIXED,  KINEMATIC };
	/*!
		Base link type. This should be specified by user.
		DYNAMIC : Move freely space. Respond to external force.(Default)
		FIXED : Fixed in space. Not respond to external force.
		KINEMATIC : Move freely space. Not respond to external force.
		Default is DYNAMIC
	*/
	BASELINKTYPE m_BaseLinkType;
	/*!
		Boolean value for self collision checking. 
		If this variable is set to be false, 
		link pairs in the same system will be excluded from the check list of collision detection.
		This should be specified by user.
		Default is false.
	*/
	bool	m_IsSelfCollision;
	/*!
		Set base link.
	*/	
	void	SetBaseLink(srLink* v);
	/*!
		Set base link type.
	*/	
	void	SetBaseLinkType(BASELINKTYPE v);
	/*!
		Set self collision.
	*/
	void	SetSelfCollision(bool v = true);
	/*!
		Get base link.
	*/
	srLink*	GetBaseLink();
	/*!
		Get base link type.
	*/
	BASELINKTYPE GetBaseLinkType();
	/*!
		Get self collision boolean value.
	*/
	bool	IsSelfCollision();

	/*!
		Update COM & ZMP.
		Argument is gravity.
	 */
	void	UpdateCOMZMP(Vec3& g);
	/*!
		Get Center of Mass of a whole body.
	 */
	Vec3&	GetCOM();
	/*!
		Get Zero Moment Point(ZMP) of a whole body.
	 */
	Vec3&	GetZMP();

	/*!
		Back up initial state of system.
	 */
	void	BackupInitState();
	/*!
		Restore initial state of system.
	 */
	void	RestoreInitState();

	/*!
		List of link entities in system.
	*/
	_array<srLink*>			m_KIN_Links;
	/*!
		List of joint entities in system.
	*/
	_array<srJoint*>		m_KIN_Joints;
	/*!
		List of collision entities in system.
	*/
	_array<srCollision*>	m_KIN_Collisions;
	/*!
		List of sensor entities in system.
	*/
	_array<srSensor*>		m_KIN_Sensors;

	/*!
		COM of a robot.
	 */
	Vec3	m_COM;
	/*!
		ZMP of a robot.
	 */
	Vec3	m_ZMP;

	//--- Lonely System
	/*!
		Boolean value for lonely system. Lonely system is a system with only base link.
	*/
	bool		m_IsLonelySystem;
	/*!
		Link of lonely system. this is the same as base link.
	*/
	srLink *	m_L_Link;
	/*!
		Inverse inertia matrix of lonely system link. This is not used yet.
	*/
	AInertia	m_L_Link_InvInertia;
	/*!
		Inertia matrix of lonely system link.
	*/
	AInertia	m_L_Link_Inertia;
	/*!
		Union-find algorithm ID. Union-find algorithm is used to find island
	*/
	srSystem *	m_UF_id;
	/*!
		Union-find algorithm size
	*/
	int			m_UF_sz;
	/*!
		Union-find algorithm index.
	*/
	int			m_UF_idx;

	/*!
		Flag for impulse test. If constraint forces are exerted, flag is on. 
		Flag is off at the beginning of every time step.
	*/
	bool		m_bExcited;

	//--- KIN
	void		KIN_ValidateSystem();
	void		KIN_Initialize();
	void		KIN_UpdateFrame_All_The_Entity();			// w/ Exp(S*q)
	void		KIN_UpdateFrame_All_The_Entity_Light();		// w/o Exp(S*q)

	void		KIN_LinkFramePropagation();
	void		KIN_UpdateFrame_CollsionEntity();
	void		KIN_UpdateFrame_SensorEntity();
	void		KIN_UpdateFrame_JointEntity();

	//--- DIFFKIN
	void		DIFFKIN_Initialize();

	void		DIFFKIN_IntegratePosition(sr_real & _step);
	void		(srSystem::*m_pfn_diffkin_integrateposition)(sr_real & _step);
	void		__FS_IntegratePosition_Euler(sr_real & _step);
	void		__L_IntegratePosition_Euler(sr_real & _step);

	void		DIFFKIN_LinkVelocityPropagation();
	void		(srSystem::*m_pfn_diffkin_linkvelocitypropagation)();
	void		__FS_LinkVelocityPropagation();
	void		__L_LinkVelocityPropagation();



	//--- DYN
	void		DYN_Initialize();

	void		DYN_ForwardDynamics_Set00(sr_real & _step);
	void		(srSystem::*m_pfn_dyn_forwarddynamics_set00)(sr_real & _step);
	void		_FS_ForwardDynamics_Set00(sr_real & _step);
	void		_L_ForwardDynamics_Set00(sr_real & _step);

	void		__FS_UpdateArticulatedInertia();
	void		__FS_UpdateBiasforce();
	void		__FS_UpdateAcceleration();
	void		__FS_IntegrateVelocity_Euler(sr_real & _step);


	void		__L_UpdateBiasforce();
	void		__L_UpdateAcceleration();
	void		__L_IntegrateVelocity_Euler(sr_real & _step);

	void		DYN_ImpulseDynamics_Set00(sr_real & _fps);
	void		(srSystem::*m_pfn_dyn_impulsedynamics_set00)(sr_real & _fps);
	void		_FS_Impulsedynamics_set00(sr_real & _fps);
	void		_L_Impulsedynamics_set00(sr_real & _fps);

	void		__FS_UpdateBiasImpulse();
	void		__FS_UpdateDelVelocity(sr_real & _fps);
	void		__FS_UpdateVelocity_with_DelVelocity();
	void		__FS_UpdateAcceleration_with_DelVelocity(sr_real & _fps);

	void		__L_UpdateDelVelocity();
	void		__L_UpdateVelocity_with_DelVelocity();
	void		__L_UpdateAcceleration_with_DelVelocity(sr_real & _fps);

	void		DYN_IntegratePosition_Set00(sr_real & _step);
	void		(srSystem::*m_pfn_dyn_integrateposition_set00)(sr_real & _step);
	void		_FS_IntegratePosition_Set00(sr_real & _step);
	void		_L_IntegratePosition_Set00(sr_real & _step);

	//-- Union-Find
	void		UF_Reset();

	//-- On Impulse Test
	void		ExciteSystem();
	void		UnExciteSystem();

	//-- Impulse Test
	void		FS_Reset_Bias_T(void);
	void		FS_UpdateBiasImpulse(srLink * pMass, const dse3& imp);
	void		FS_UpdateBiasImpulse(srLink * pMass);
	void		FS_UpdateDelVelocity();

	void		L_UpdateDelVelocity(const dse3& imp);

	//--- Not used
	//-- ConstraintForce Reset
	void		FS_Reset_ConstraintImpulse();
	void		L_Reset_ConstraintImpulse();
	//-- Error Correction
	void		ResetPosErrVelocity(void);
	void		FS_MassPosErrVelocityPropagation(void);
	void		FS_IntegratePosErrVelocity_plus_DelVelocity(void);
	void		FS_IntegratePosition_PosErrVel_Euler(sr_real & _step);
	void		FS_IntegratePosition_PosErrVel_plus_Velocity_Euler(sr_real & _step);


	//-- 2009.03.12 adding closed loop (under construction)
	struct linkpair_for_closedloop
	{
		srLink * LeftLink;
		srLink * RightLink;
		SE3		 RelativeFrame;
	};
	_array<linkpair_for_closedloop>	m_linkpairs_for_closedloop;
	void		MakeClosedLoop(srLink * pLeftLink, srLink * pRightLink, SE3 RelativeFrame = SE3());

protected:
};
#endif

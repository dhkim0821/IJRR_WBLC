#ifndef	SRLIB_JOINT
#define SRLIB_JOINT

#include "srDyn/srEntity.h"
#include "srDyn/srState.h"

class srLink;
class srState;
class srSystem;

/*!
	\class srJoint
	\brief Class represents joint object.

	srJoint is an abstract class.
*/
class srJoint : public srEntity
{
public:
	enum ACTTYPE { PASSIVE, TORQUE, VELOCITY, HYBRID };
	enum JOINTTYPE { REVOLUTE, PRISMATIC, UNIVERSAL, WELD, BALL };

	/*!
		Actuator type. This should be specified by user.
		Default is PASSIVE.

		PASSIVE : User cannot apply torque on joint. 
				  torque is generated according to spring and damping coefficent.
		TORQUE : User can apply torque on joint directly.
				 i.g) GetStatePtr()->SetCommand(real * JointTorque)
		VELOCITY : User can set desired velocity of joint . To use VELOCITY actuator, user must specify finite torque limit of joint.
				 i.g) i.g) GetStatePtr()->SetCommand(real * DesiredVelocity)
		HYBRID : User can set acceleration of joint.
				 i.g) i.g) GetStatePtr()->SetCommand(real * Acceleration)

		Command on joint is not reset to zero during the simulation.
		User must specify the commands on joints every time step.
	*/
	ACTTYPE	m_ActType;
	/*!
		Parent link in hierarchy structure. This should be specified by user.
	*/
	srLink*	m_ParentLink;
	/*!
		Child link in hierarchy structure. This should be specified by user.
	*/
	srLink*	m_ChildLink;
	/*!
		Relative frame from parent link to joint frame. This should be specified by user.
	*/
	SE3		m_ParentLinkToJoint;
	/*!
		Relative frame from child link to joint frame. This should be specified by user.
	*/
	SE3		m_ChildLinkToJoint;
	/*!
		Priority index. Default is 0.
		Index for which link is the basis to update joint frame.
		Joint axis is inverted when priority index is 1.
		0 is for parent link. 1 is for child link.
		Default is 0.
	*/
	int		m_PriorityIndex;
	/*!
		Boolean value for device on-off.
		This is used for actuator type HYBRID and VELOCITY. 
		When device is off, actuator ignores command and behaves like passive type.
	*/
	bool	m_IsDeviceOn;

public:
	ACTTYPE	GetActType();
	void	SetActType(ACTTYPE );

	virtual JOINTTYPE	GetType();

	void	SetParentLink(srLink* );
	void	SetChildLink(srLink* );

	SE3&	GetParentLinkFrame();
	SE3&	GetChildLinkFrame();
	void	SetParentLinkFrame(SE3 v = SE3() );
	void	SetChildLinkFrame(SE3 v = SE3() );

	void	SetPriorityIndex(int idx = 0);

	bool	IsDeviceOn();
	virtual void	SetDeviceOnOff(bool onoff = true) = 0;

public:
	/*!
		Constructor.
	*/
	srJoint();
	/*!
		Destructor.
	*/
	virtual ~srJoint();
	/*!
		Parent system to which joint entity belongs.
	*/
	srSystem * m_pSystem;
	/*!
		Joint type. 
		REVOLUTE, PRISMATIC, UNIVERSAL, WELD.
	*/
	JOINTTYPE m_JointType;


public:


	SE3		m_FS_SE3;
	se3		m_FS_LocalVelocity;
	dse3	m_FS_Force;
	SE3		m_FS_Force_SE3;


	/*!
		Get state pointer.
	*/
	virtual srState* GetStatePtr() = 0;

	virtual void Initialize() = 0;
	virtual void FS_UpdateForce(const dse3& F) = 0;
	virtual void FS_UpdateAIS_K(const AInertia& AI) = 0;
	virtual void FS_UpdateAIS_K_P(AInertia& AIjari, const AInertia& AI) = 0;
	virtual void FS_UpdateBiasImp(dse3& Cias, const dse3& Bias) = 0;
	virtual void FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V) = 0;
	virtual void FS_UpdateLocalDelVel(se3& jari, const se3& DV) = 0;
	virtual void FS_UpdateLocalAcc(se3& jari, const se3& DV) = 0;
	virtual dse3 GetFT();

	void			UpdateFrame(void);
	void			Set_UpdateFrame_Ftn();
	void			(srJoint::*m_pfn_updateframe)();
	void			_updateframe_Primary();
	void			_updateframe_Secondary();
	void			_updateframe_DoNothing();

	virtual SE3&	FS_Transform(void)								= 0;
	virtual se3&	FS_UpdateLocalVelocity(void)					= 0;
	virtual se3&	FS_UpdatePosErrorLocalVelocity(void)			= 0;

	virtual	void	FS_SetScrew(int i) = 0;
	virtual void	FS_ResetT(void) = 0;

protected:
};

#endif

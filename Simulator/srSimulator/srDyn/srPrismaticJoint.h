#ifndef	SRLIB_PRISMATIC_JOINT
#define SRLIB_PRISMATIC_JOINT

#include "srDyn/srJoint.h"
#include "srDyn/srState.h"

/*!
	\class srPrismaticJoint
	\brief Class represents prismatic joint.
*/
class srPrismaticJoint : public srJoint
{
public:
	/*!
		Boolean value for limit of angle of prismatic joint. This should be specified by user.
		Valid when actuator type is PASSIVE, TORQUE or VELOCITY.
		Default is true.
	*/
	bool	m_IsPosLimited;

	/*!
		Limits of prismatic joint angle. m_PosLimit[0] is lower limit and m_PosLimit[1] is upper limit.
		Upper limit must be greater than lower limit. This should be specified by user.
		Defaults are (-60, 60). Unit is degree.
	*/
	sr_real	m_PosLimit[2];
	/*!
		Limits of prismatic joint torque. Valid when actuator type is VELOCITY.
		m_TorqueLimit[0] is lower limit and m_TorqueLimit[1] is upper limit.
		It is recommended that upper limit is set to be finite positive value and lower limit is set to be finite negative value.
		This should be specified by user.
		Default is (-100, 100).
	*/
	sr_real	m_TorqueLimit[2];

	/*! 
		Offset angle of prismatic joint. Spring force is zero when angle of joint equals offset angle.
		Valid when actuator type is PASSIVE. This should be specified by user.
		Default is 0.0.
	*/
	sr_real	m_rOffset;
	/*! 
		Spring coefficient of prismatic joint.
		Valid when actuator type is PASSIVE. This should be specified by user.
		This must be greater than or equal to zero.
		Default is 0.0.
	*/
	sr_real	m_rK;
	/*!
		Damping coefficient of prismatic joint.
		Valid when actuator type is PASSIVE. This should be specified by user.
		This must be greater than or equal to zero.
		Default is 0.01.
	*/
	sr_real	m_rC;
	/*!
		Get position limit boolean value.
	*/
	bool	IsPostionLimited();
	/*!
		Set position limit boolean value.
	*/
	void	MakePositionLimit(bool v = true);
	/*!
		Get lower limit of joint angle.
	*/
	sr_real	GetPositionLowerLimit();
	/*!
		Get upper limit of joint angle.
	*/
	sr_real	GetPositionUpperLimit();
	/*!
		Set limits of joint angle . upper limit must be greater than lower limit.
	*/
	void	SetPositionLimit(sr_real lowerlimit, sr_real upperlimit);
	/*!
		Get lower limit of joint torque.
	*/
	sr_real	GetTorqueLowerLimit();
	/*!
		Get upper limit of joint torque.
	*/
	sr_real	GetTorqueUpperLimit();
	/*!
		Set limits of joint torque . upper limit must be greater than lower limit.
	*/
	void	SetTorqueLimit(sr_real lowerlimit, sr_real upperlimit);
	/*!
		Get offset.
	*/
	sr_real	GetOffset();
	/*!
		Set offset.
	*/
	void	SetOffset(sr_real v);
	/*!
		Get spring coefficient.
	*/
	sr_real	GetSpringCoeff();
	/*!
		Set spring coefficient. This must be greater than or equal to zero.
	*/
	void	SetSpringCoeff(sr_real v);
	/*!
		Get damping coefficient.
	*/
	sr_real	GetDampingCoeff();
	/*!
		Set damping coefficient. This must be greater than or equal to zero.
	*/
	void	SetDampingCoeff(sr_real v);
	/*!
		Turn on or off the actuator. 
		Device on-off is valid when actuator type is HYBRID or VELOCITY.
		This can be called during simulation.
	*/
	virtual void	SetDeviceOnOff(bool onoff = true);
	/*!
		Get prismatic joint state.
	*/
	srPrismaticState& GetPrismaticJointState();


public:
	/*!
		Constructor.
	*/
	srPrismaticJoint();
	/*!
		Destructor.
	*/
	virtual ~srPrismaticJoint();
	/*!
		Prismatic joint state.
	*/
	srPrismaticState	m_State;
	/*!
		Axis of prismatic joint. This is pre-fixed as z-axis.
	*/
	se3		m_Axis;


	se3		m_FS_Screw;
	dse3	m_FS_AIS;
	sr_real	m_FS_K;
	sr_real	m_FS_T;
	se3		m_FS_W;

	virtual srState*	GetStatePtr();

	virtual void Initialize();
	virtual void FS_UpdateForce(const dse3& F);
	virtual void FS_UpdateAIS_K(const AInertia& AI);
	virtual void FS_UpdateAIS_K_P(AInertia& AIjari, const AInertia& AI);
	virtual void FS_UpdateBiasImp(dse3& Cias, const dse3& Bias);
	virtual void FS_UpdateBiasforce(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	virtual void FS_UpdateLocalDelVel(se3& jari, const se3& DV);
	virtual void FS_UpdateLocalAcc(se3& jari, const se3& DV);

	virtual SE3&	FS_Transform(void);
	virtual se3&	FS_UpdateLocalVelocity(void);
	virtual se3&	FS_UpdatePosErrorLocalVelocity(void);

	virtual void	FS_SetScrew(int i);
	virtual void	FS_ResetT(void);

protected:
	// Actuator function
	void	(srPrismaticJoint::*m_pfnFS_UpdateForce)(const dse3& F);
	void	(srPrismaticJoint::*m_pfnFS_UpdateAIS_K)(const AInertia& AI);
	void	(srPrismaticJoint::*m_pfnFS_UpdateAIS_K_P)(AInertia& AIjari, const AInertia& AI);
	void	(srPrismaticJoint::*m_pfnFS_UpdateBiasImp)(dse3& Cias, const dse3& Bias);
	void	(srPrismaticJoint::*m_pfnFS_UpdateBiasforce)(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	(srPrismaticJoint::*m_pfnFS_UpdateLocalDelVel)(se3& jari, const se3& DV);
	void	(srPrismaticJoint::*m_pfnFS_UpdateLocalAcc)(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Passive(const dse3& F);
	void	_FS_UpdateAIS_K_Passive(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Passive(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Passive(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Passive(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Passive(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Passive(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Torque(const dse3& F);
	void	_FS_UpdateAIS_K_Torque(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Torque(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Torque(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Torque(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Torque(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Torque(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Servo(const dse3& F);
	void	_FS_UpdateAIS_K_Servo(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Servo(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Servo(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Servo(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Servo(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Servo(se3& jari, const se3& DV);

	void	_FS_UpdateForce_Hybrid(const dse3& F);
	void	_FS_UpdateAIS_K_Hybrid(const AInertia& AI);
	void	_FS_UpdateAIS_K_P_Hybrid(AInertia& AIjari, const AInertia& AI);
	void	_FS_UpdateBiasImp_Hybrid(dse3& Cias, const dse3& Bias);
	void	_FS_UpdateBiasforce_Hybrid(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	_FS_UpdateLocalDelVel_Hybrid(se3& jari, const se3& DV);
	void	_FS_UpdateLocalAcc_Hybrid(se3& jari, const se3& DV);
};

#endif

#ifndef	SRLIB_UNIVERSAL_JOINT
#define SRLIB_UNIVERSAL_JOINT

#include "srDyn/srJoint.h"

/*!
	\class srUniversalJoint
	\brief Class represents universal joint.
*/
class srUniversalJoint : public srJoint
{
public:
	/*!
		Boolean values for limit of angles of universal joint. This should be specified by user.
		Valid when actuator type is PASSIVE, TORQUE or VELOCITY.
		Default is (true, true).
	*/
	bool	m_IsPosLimited[2];

	/*!
		Limits of first angle of universal joint. m_PosLimit_1[0] is lower limit and m_PosLimit_1[1] is upper limit.
		Upper limit must be greater than lower limit. This should be specified by user.
		Defaults are (-60, 60). Unit is degree.
	*/
	sr_real	m_PosLimit_1[2];
	/*!
		Limits of second angle of universal joint. m_PosLimit_2[0] is lower limit and m_PosLimit_2[1] is upper limit.
		Upper limit must be greater than lower limit. This should be specified by user.
		Defaults are (-60, 60). Unit is degree.
	*/
	sr_real	m_PosLimit_2[2];
	/*!
		Limits of first torque of universal joint. Valid when actuator type is VELOCITY.
		m_TorqueLimit_1[0] is lower limit and m_TorqueLimit_1[1] is upper limit.
		It is recommended that upper limit is set to be finite positive value and lower limit is set to be finite negative value.
		This should be specified by user.
		Default is (-100, 100).
	*/
	sr_real	m_TorqueLimit_1[2];
	/*!
		Limits of second torque of universal joint. Valid when actuator type is VELOCITY.
		m_TorqueLimit_2[0] is lower limit and m_TorqueLimit_2[1] is upper limit.
		It is recommended that upper limit is set to be finite positive value and lower limit is set to be finite negative value.
		This should be specified by user.
		Default is (-100, 100).
	*/
	sr_real	m_TorqueLimit_2[2];
	/*! 
		Offset angles of universal joint. Spring force is zero when angle of joint equals offset angle.
		m_rOffset[0] is for first angle of joint. m_rOffset[1] is for second angle of joint.
		Valid when actuator type is PASSIVE. This should be specified by user.
		Default is (0.0, 0.0).
	*/
	sr_real			 m_rOffset[2];
	/*! 
		Spring coefficients of universal joint.
		m_rK[0] is for first angle of joint. m_rK[1] is for second angle of joint.
		Valid when actuator type is PASSIVE. This should be specified by user.
		This must be greater than or equal to zero.
		Default is (0.0, 0.0)
	*/
	sr_real			 m_rK[2];
	/*! 
		Damping coefficients of universal joint.
		m_rC[0] is for first angle of joint. m_rC[1] is for second angle of joint.
		Valid when actuator type is PASSIVE. This should be specified by user.
		This must be greater than or equal to zero.
		Default is (0.01, 0.01).
	*/
	sr_real			 m_rC[2];
	/*!
		Get position limit boolean value.
		Get boolean value for the first angle of joint when idx equals 0.
		Get boolean value for the second angle of joint when idx equals 1.
	*/
	bool	IsPostionLimited(int idx = 0);
	/*!
		Set position limit boolean value.
		Set boolean value for the first angle of joint when idx equals 0.
		Set boolean value for the second angle of joint when idx equals 1.
		Set boolean values for both angles of joint when idx equals 2.
	*/
	void	MakePositionLimit(bool v = true, int idx = 2);
	/*!
		Get lower limit of joint angle.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetPositionLowerLimit(int idx = 0);
	/*!
		Get upper limit of joint angle.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetPositionUpperLimit(int idx = 0);
	/*!
		Set limits of joint angle . Upper limit must be greater than lower limit.
		Set sr_real values for the first angle of joint when idx equals 0.
		Set sr_real values for the second angle of joint when idx equals 1.
		Set sr_real values for both angles of joint when idx equals 2.
	*/
	void	SetPositionLimit(sr_real lowerlimit, sr_real upperlimit, int idx = 2);
	/*!
		Get lower limit of joint torque.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetTorqueLowerLimit(int idx = 0);
	/*!
		Get upper limit of joint torque.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetTorqueUpperLimit(int idx = 0);
	/*!
		Set limits of joint torque . Upper limit must be greater than lower limit.
		Set sr_real values for the first angle of joint when idx equals 0.
		Set sr_real values for the second angle of joint when idx equals 1.
		Set sr_real values for both angles of joint when idx equals 2.
	*/
	void	SetTorqueLimit(sr_real lowerlimit, sr_real upperlimit, int idx = 2);
	/*!
		Get offset.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetOffset(int idx = 0);
	/*!
		Set offset.
		Set sr_real values for the first angle of joint when idx equals 0.
		Set sr_real values for the second angle of joint when idx equals 1.
	*/
	void	SetOffset(sr_real v, int idx = 0);
	/*!
		Get spring coefficient.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetSpringCoeff(int idx = 0);
	/*!
		Set spring coefficient. This must be greater than or equal to zero.
		Set sr_real values for the first angle of joint when idx equals 0.
		Set sr_real values for the second angle of joint when idx equals 1.
		Set sr_real values for both angles of joint when idx equals 2.
	*/
	void	SetSpringCoeff(sr_real v, int idx = 2);
	/*!
		Get damping coefficient.
		Get sr_real value for the first angle of joint when idx equals 0.
		Get sr_real value for the second angle of joint when idx equals 1.
	*/
	sr_real	GetDampingCoeff(int idx = 0);
	/*!
		Set damping coefficient. This must be greater than or equal to zero.
		Set sr_real values for the first angle of joint when idx equals 0.
		Set sr_real values for the second angle of joint when idx equals 1.
		Set sr_real values for both angles of joint when idx equals 2.
	*/
	void	SetDampingCoeff(sr_real v, int idx = 2);
	/*!
		Turn on or off the actuator. 
		Device on-off is valid when actuator type is HYBRID or VELOCITY.
		This can be called during simulation.
	*/
	virtual void	SetDeviceOnOff(bool onoff = true);
	/*!
		Get universal joint state.
	*/
	srUniversalState& GetUniversalJointState();

public:
	/*!
		Constructor.
	*/
	srUniversalJoint();
	/*!
		Destructor.
	*/
	virtual ~srUniversalJoint();
	/*!
		Universal joint state.
	*/
	srUniversalState	m_State;

	int				 m_Idx_State[2];
	/*!
		First axis of universal joint. This is pre-fixed as x-axis.
	*/
	se3				 m_Axis1;
	/*!
		Second axis of universal joint. This is pre-fixed as z-axis.
	*/
	se3				 m_Axis2;


	se3				 m_FS_Screw1, m_FS_Screw2;
	SE3				 m_FS_SE3_1, m_FS_SE3_2;
	se3				 m_FS_LocalVelocity_1, m_FS_LocalVelocity_2;
	dse3			 m_FS_AIS_1, m_FS_AIS_2;
	sr_real			 m_FS_K_1, m_FS_K_2;
	AInertia		 m_FS_AI_med;
	sr_real			 m_FS_T1, m_FS_T2;
	se3				 m_FS_W1, m_FS_W2;

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
	void	(srUniversalJoint::*m_pfnFS_UpdateForce)(const dse3& F);
	void	(srUniversalJoint::*m_pfnFS_UpdateAIS_K)(const AInertia& AI);
	void	(srUniversalJoint::*m_pfnFS_UpdateAIS_K_P)(AInertia& AIjari, const AInertia& AI);
	void	(srUniversalJoint::*m_pfnFS_UpdateBiasImp)(dse3& Cias, const dse3& Bias);
	void	(srUniversalJoint::*m_pfnFS_UpdateBiasforce)(dse3& Cias, const dse3& Bias, const AInertia& AI, const se3& V);
	void	(srUniversalJoint::*m_pfnFS_UpdateLocalDelVel)(se3& jari, const se3& DV);
	void	(srUniversalJoint::*m_pfnFS_UpdateLocalAcc)(se3& jari, const se3& DV);

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

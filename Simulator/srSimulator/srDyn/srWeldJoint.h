#ifndef	SRLIB_WELD_JOINT
#define SRLIB_WELD_JOINT

#include "srDyn/srJoint.h"


/*!
	\class srWeldJoint
	\brief Class represents weld joint.
*/
class srWeldJoint : public srJoint
{
public:
	/*!
		Turn on or off the actuator.
		Do not work in weld joint.
	*/
	virtual void	SetDeviceOnOff(bool onoff = true);

public:
	/*!
		Constructor.
	*/
	srWeldJoint();
	/*!
		Destructor.
	*/
	virtual ~srWeldJoint();
	/*!
		Weld joint state.
		No information included.
	*/
	srWeldState	m_State;

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
};

#endif


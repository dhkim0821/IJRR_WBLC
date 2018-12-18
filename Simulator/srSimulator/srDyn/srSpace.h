#ifndef	SRLIB_SPACE
#define SRLIB_SPACE

#include "srDyn/srObject.h"
#include "LieGroup/_array.h"

#include "srDyn/srState.h"
#include "srDyn/srSystem.h"
#include "srDyn/srLink.h"
#include "srDyn/srJoint.h"
#include "srDyn/srCollision.h"
#include "srDyn/srState.h"
#include "srDyn/srSensor.h"

#include "srDyn/srRevoluteJoint.h"
#include "srDyn/srPrismaticJoint.h"
#include "srDyn/srUniversalJoint.h"
#include "srDyn/srWeldJoint.h"
#include "srDyn/srBallJoint.h"

#include "srDyn/srTouchSensor.h"
#include "srDyn/srIRSensor.h"
#include "srDyn/srRangeFinder.h"

#include "srDyn/srSpring.h"
#include "srDyn/srLinearSpring.h"

#include "srDyn/srDYN.h"



#define	DEFAULT_NUMOFSUBSTEP_RENDER_TEMP		50

void	_do_nothing();
/*!
	\class srSpace
	\brief Class represents 3D space.
*/
class srSpace : public srObject
{
public:
	/*! 
		Gravity vector in 3 dimension. This should be specified by user.
		Default is (0.0, 0.0, -9.8)
	*/
	Vec3					m_Gravity;
	/*! 
		Simulation time step. This should be specified by user.
		Default is 0.001 
	*/
	sr_real					m_Timestep_dyn_fixed;
	/*!
		Number of sub-step for rendering. This should be specified by user.
		Default is 50.
	*/
	int						m_nSubstep_render_fixed;
	/*!
		List of systems in the space. This should be specified by user.
	*/
	_array<srSystem*>					m_Systems;
	/*!
		Set gravity vector.
	*/
	void	SetGravity(sr_real x, sr_real y, sr_real z);
	/*!
		Set simulation time step.
	*/
	void	SetTimestep(sr_real v);
	/*!
		Set number of sub-step for rendering
	*/
	void	SetNumberofSubstepForRendering(int v);
	/*!
		Add system to space.
	*/
	void	AddSystem(srSystem * v);
	
	/*!
		Add spring to space
	 */
	void	AddSpring(srSpring * s);


public:
	srSpace();

	/*!
		Simulation time. 
		This increases as much as time step for one step forward of dynamics simulation.
	*/
	sr_real								m_Simulation_Time;
	/*!
		Reciprocal of time step (1/timestep).
	*/
	sr_real								m_FPS_dyn_fixed;
	/*!
		List of link entities in the space.
	*/
	_array<srLink*>						m_Links;		
	/*!
		List of joint entities in the space.
	*/
	_array<srJoint*>					m_Joints;		
	/*!
		List of collision entities in the space.
	*/
	_array<srCollision*>				m_Collisions;	
	/*!
		List of sensor entities in the space.
	*/
	_array<srSensor*>					m_Sensors;		
	/*!
		List of states of joints in the space.
	*/
	_array<srState*>					m_States;
	/*!
		List of springs in the space.
	 */
	_array<srSpring*>					m_Springs;

	/*!
		Dynamics engine(constraint solver).
	*/
	srDYN								m_srDYN;

public:
	///--- Allowed to user.
	/*!
		Get simulation time.
	*/
	const sr_real				GetSimulationTime() const;
	/*!
		Set simulation time zero.
	*/
	void					ResetSimulationTime();
	/*!
		Back up initial state of all system.
	 */
	void	_BackupInitState();
	/*!
		Restore initial state of all system.
	 */
	void	_RestoreInitState();
	/*!
		Initialization for dynamics simulation.
	*/
	void	DYN_MODE_PRESTEP();
	/*!
		Initialization for kinematics simulation.
	*/
	void	KIN_MODE_PRESTEP();
	/*!
		Dynamics simulation loop.
	*/
	void	DYN_MODE_RUNTIME_SIMULATION_LOOP();
    void	DYN_MODE_RUNTIME_SIMULATION_LOOP(void* _pt);
	/*!
		Kinematics simulation loop.
		Only forward kinematics and sensor detection routines are included.
	*/
	void	KIN_MODE_RUNTIME_FORWARD_KINEMATICS();
	/*!
		Set user control function.
		User control function is function is called every time step during simulation.
	*/
	void	SET_USER_CONTROL_FUNCTION(void (*pfn)() );
    void	SET_USER_CONTROL_FUNCTION_2(void (*pfn)(void*) );
//protected:
public:
	// PRESTEP
	void	_PRESTEP_Build_ObjectPools();

	void	_PRESTEP_Collision_Setting();
	void	_PRESTEP_Sensor_Setting();

	void	_PRESTEP_DYN_Space_Setting();
	void	_PRESTEP_DYN_System_Setting();
	
	void	_PRESTEP_DYN_Spring_Setting();

	// RUNTIME
	void	_KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	void	_KIN_UpdateFrame_All_The_Sensors_All_The_Systems();
	void	_KIN_UpdateFrame_All_The_Joints_All_The_Systems();
	void	_KIN_Update_All_The_Sensors();

	void	_DIFFKIN_LinkVelocityPropagation_All_The_Systmes();
	void	_DIFFKIN_IntegratePosition_All_The_Links_All_The_Systmes();

	void	_DYN_Step_Forward_All_The_Systems();
	void	__DYN_Reset();
	void	__DYN_Before_srDYN();
	void	__DYN_After_srDYN();
	void	__DYN_Reset_Post();

	void	_Step_Simulation_Time();

	// USER CONTROL FUNCTION
	void	_USER_CONTROL();
    void    _USER_CONTROL(void* _pt);
	void	(*m_pfn_USER_CONTROL)();
    void    (*m_pfn_USER_CONTROL_2)(void*);

protected:
};

#endif

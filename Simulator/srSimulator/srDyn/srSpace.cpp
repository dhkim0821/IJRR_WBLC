#include "srDyn/srSpace.h"

#include "srDyn/srIRSensor.h"
#include "srDyn/srRangeFinder.h"
#include "srDyn/srTouchSensor.h"

#include <srConfiguration.h>


srSpace::srSpace()
{
	m_Gravity = Vec3(0.0, 0.0, -9.8);
	m_Timestep_dyn_fixed = 0.001;

	m_nSubstep_render_fixed = DEFAULT_NUMOFSUBSTEP_RENDER_TEMP;

	m_Simulation_Time = 0.0;

	m_pfn_USER_CONTROL = _do_nothing;

	m_FPS_dyn_fixed = 1000;
}

void srSpace::SetGravity(sr_real x, sr_real y, sr_real z)
{
	m_Gravity = Vec3(x,y,z);
}

void srSpace::SetTimestep(sr_real v)
{
	if ( v > 0.0)
		m_Timestep_dyn_fixed = v;
}

void srSpace::SetNumberofSubstepForRendering(int v)
{
	if ( v >= 1)
		m_nSubstep_render_fixed = v;
}

void srSpace::AddSystem(srSystem * v)
{
	if(m_Systems.find(v) == -1)
		m_Systems.add_tail(v);
}

void srSpace::AddSpring(srSpring * s)
{
	if(m_Springs.find(s) == -1)
		m_Springs.add_tail(s);
}

void srSpace::_BackupInitState()
{
	for(int i = 0 ; i < m_Systems.get_size() ; ++i)
		m_Systems[i]->BackupInitState();
}

void srSpace::_RestoreInitState()
{
	for(int i = 0 ; i < m_Systems.get_size() ; ++i)
		m_Systems[i]->RestoreInitState();
}

const sr_real srSpace::GetSimulationTime() const
{
	return m_Simulation_Time;
}

void srSpace::ResetSimulationTime()
{
	m_Simulation_Time = 0.0;
}

void srSpace::DYN_MODE_PRESTEP()
{
	_PRESTEP_Build_ObjectPools();

	_PRESTEP_Collision_Setting();
	_PRESTEP_Sensor_Setting();

	_PRESTEP_DYN_Space_Setting();
	_PRESTEP_DYN_System_Setting();

	_KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	_DIFFKIN_LinkVelocityPropagation_All_The_Systmes();

	_BackupInitState();

	_PRESTEP_DYN_Spring_Setting();

	m_srDYN.PRESTEP_MARK9(m_Systems, m_Links, m_Joints, m_Timestep_dyn_fixed, m_FPS_dyn_fixed);
}

void srSpace::KIN_MODE_PRESTEP()
{
	_PRESTEP_Build_ObjectPools();

	_PRESTEP_Collision_Setting();
	_PRESTEP_Sensor_Setting();

	_KIN_UpdateFrame_All_The_Entity_All_The_Systems();

	_BackupInitState();
}

void srSpace::SET_USER_CONTROL_FUNCTION(void (*pfn)())
{
	m_pfn_USER_CONTROL = pfn;
}
void srSpace::SET_USER_CONTROL_FUNCTION_2(void (*pfn)(void*))
{
	m_pfn_USER_CONTROL_2 = pfn;
}

void srSpace::DYN_MODE_RUNTIME_SIMULATION_LOOP()
{
	for (int i = 0 ; i < m_nSubstep_render_fixed  ; i++)
    {
      _DYN_Step_Forward_All_The_Systems();
      _KIN_UpdateFrame_All_The_Sensors_All_The_Systems();
      _KIN_UpdateFrame_All_The_Joints_All_The_Systems();

      _KIN_Update_All_The_Sensors();

      _Step_Simulation_Time();

      _USER_CONTROL();
    }
}

void srSpace::DYN_MODE_RUNTIME_SIMULATION_LOOP(void* _pt)
{
  for (int i = 0 ; i < m_nSubstep_render_fixed  ; i++)
    {
      _DYN_Step_Forward_All_The_Systems();
      _KIN_UpdateFrame_All_The_Sensors_All_The_Systems();
      _KIN_UpdateFrame_All_The_Joints_All_The_Systems();
        
      _KIN_Update_All_The_Sensors();
        
      _Step_Simulation_Time();
        
      _USER_CONTROL(_pt);
    }

}


void srSpace::KIN_MODE_RUNTIME_FORWARD_KINEMATICS()
{
	for (int i = 0 ; i < m_nSubstep_render_fixed  ; i++)
    {
      _KIN_UpdateFrame_All_The_Entity_All_The_Systems();
      _KIN_Update_All_The_Sensors();
      _Step_Simulation_Time();
      _USER_CONTROL();
    }
}

//////////////////////////////////////////////////////////////////////////
// PRESETP
//////////////////////////////////////////////////////////////////////////

void srSpace::_PRESTEP_Build_ObjectPools()
{
	// S'pse m_Systems are valid.
	
	m_Links.clear();
	m_Joints.clear();
	m_Collisions.clear();
	m_Sensors.clear();
	m_States.clear();

	int i, j, iCount, jCount;
	srSystem * pSystem;
	srLink	* pLink;
	srJoint	* pJoint;
	srCollision	* pCollision;
	srSensor	* pSensor;
	srState		* pState;

	// System Pool
	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pSystem = m_Systems[i];

      // TEMP.
      pSystem->KIN_ValidateSystem();

      // Link Pool
      jCount = pSystem->m_KIN_Links.get_size();
      for (j = 0 ; j < jCount ; j++)
        {
          pLink = pSystem->m_KIN_Links[j];
          m_Links.add_tail(pLink);
        }

      // Joint Pool
      // State Pool
      jCount = pSystem->m_KIN_Joints.get_size();
      for (j = 0 ; j < jCount ; j++)
        {
          pJoint = pSystem->m_KIN_Joints[j];
          m_Joints.add_tail(pJoint);

          pState = pJoint->GetStatePtr();
          m_States.add_tail(pState);
        }

      // Collision Pool
      jCount = pSystem->m_KIN_Collisions.get_size();
      for (j = 0 ; j < jCount ; j++)
        {
          pCollision = pSystem->m_KIN_Collisions[j];
          m_Collisions.add_tail(pCollision);
        }

      // Sensor Pool
      jCount = pSystem->m_KIN_Sensors.get_size();
      for (j = 0 ; j < jCount ; j++)
        {
          pSensor = pSystem->m_KIN_Sensors[j];
          m_Sensors.add_tail(pSensor);
        }
    }
}

// KIN
void srSpace::_PRESTEP_Collision_Setting()
{
	int i, iCount;
	srCollision * pCollision;

	//--- Collisions
	// : Update Bounding Sphere, Set SensorDetection Ftn Ptr
	iCount = m_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pCollision = m_Collisions[i];
      pCollision->UpdateBoundingRadius();
      pCollision->UpdateRayDetectionFtn();
      pCollision->UpdateTouchDetectionFtn();
      pCollision->UpdateSonarDetectionFtn();
    }
}

void srSpace::_PRESTEP_Sensor_Setting()
{
	int i, j, iCount, jCount;

	srSensor * pSensor;
	srCollision * pCollision;

	iCount = m_Sensors.get_size();
	jCount = m_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pSensor = m_Sensors[i];
      pSensor->Initialize();

      if ( srSensor::IRSENSOR == pSensor->m_SensorType )
        {
          srIRSensor * pIRSensor = (srIRSensor*)pSensor;
          pIRSensor->m_Objects.clear();
          for (j = 0 ; j < jCount ; j++)
            {
              pCollision = m_Collisions[j];
              if ( pCollision->m_pLink != pIRSensor->m_pLink )
                pIRSensor->AddObject(pCollision);
            }
        }
      else if ( srSensor::RANGEFINDER == pSensor->m_SensorType )
        {
          srRangeFinder * pRangeFinder = (srRangeFinder*)pSensor;
          pRangeFinder->m_Objects.clear();
          for (j = 0 ; j < jCount ; j++)
            {
              pCollision = m_Collisions[j];
              if ( pCollision->m_pLink != pRangeFinder->m_pLink )
                pRangeFinder->AddObject(pCollision);
            }
        }
      else if ( srSensor::TOUCHSENSOR == pSensor->m_SensorType )
        {
          srTouchSensor * pTouchSensor = (srTouchSensor*)pSensor;
          pTouchSensor->m_Objects.clear();
          for (j = 0 ; j < jCount ; j++)
            {
              pCollision = m_Collisions[j];
              if ( pCollision->m_pLink != pTouchSensor->m_pLink )
                pTouchSensor->AddObject(pCollision);
            }
        }
    }
}

// DYN
void srSpace::_PRESTEP_DYN_Space_Setting()
{
	//--- Space
	ResetSimulationTime();
	m_FPS_dyn_fixed = 1.0/m_Timestep_dyn_fixed;
}

void srSpace::_PRESTEP_DYN_System_Setting()
{
	int i, iCount;
	srSystem * pSystem;

	//--- Systems
	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pSystem = m_Systems[i];

      pSystem->DYN_Initialize();
    }
}

void srSpace::_PRESTEP_DYN_Spring_Setting()
{
	int i, iCount;
	srSpring * pSpring;
	iCount = m_Springs.get_size();
	for(i = 0 ; i < iCount ; i++)
    {	
      pSpring = m_Springs[i];
		
      pSpring->Initialize();
    }
}

//////////////////////////////////////////////////////////////////////////
// RUNTIME
//////////////////////////////////////////////////////////////////////////

// KIN
inline void srSpace::_KIN_UpdateFrame_All_The_Entity_All_The_Systems()
{
	int i, iCount;

	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Systems[i]->KIN_UpdateFrame_All_The_Entity();
    }
}

void srSpace::_KIN_UpdateFrame_All_The_Sensors_All_The_Systems()
{
	int i, iCount;

	iCount = m_Sensors.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Sensors[i]->UpdateFrame();
    }
}

inline void srSpace::_KIN_UpdateFrame_All_The_Joints_All_The_Systems()
{
	int i, iCount;

	iCount = m_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Joints[i]->UpdateFrame();
    }
}

inline void srSpace::_KIN_Update_All_The_Sensors()
{
	int i, iCount;

	iCount = m_Sensors.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Sensors[i]->Detect();		
    }
}

// DIFFKIN
void srSpace::_DIFFKIN_LinkVelocityPropagation_All_The_Systmes()
{
	int i, iCount;

	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Systems[i]->DIFFKIN_LinkVelocityPropagation();
    }
}

void srSpace::_DIFFKIN_IntegratePosition_All_The_Links_All_The_Systmes()
{
	int i, iCount;

	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Systems[i]->DIFFKIN_IntegratePosition(m_Timestep_dyn_fixed);
    }
}

// DYN
inline void srSpace::_DYN_Step_Forward_All_The_Systems()
{
	__DYN_Reset();
	__DYN_Before_srDYN();
	m_srDYN.RUNTIME_MARK9();
	__DYN_After_srDYN();
	__DYN_Reset_Post();
}

inline void srSpace::__DYN_Reset()
{
	int i, iCount;
	srLink * pLink;
	srState * pState;
	srSystem * pSystem;
	srSpring * pSpring;

	//--- Links
	iCount = m_Links.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pLink = m_Links[i];

      // Reset external force and add Damping (for preventing divergence)
      pLink->m_ExtForce = - pLink->m_Damping*pLink->m_Vel;

      // Add gravity force to external force
      pLink->m_ExtForce += pLink->m_Inertia * InvAd(pLink->m_Frame, m_Gravity);

      //// Add user external force to external force
      pLink->m_ExtForce += pLink->m_UserExtForce;

      // Constraint Force Reset
      pLink->m_ConstraintImpulse = 0.0;
    }
	
	//--- Spring force
	iCount = m_Springs.get_size();
	for(i = 0 ; i < iCount ; ++i)
    {
      pSpring = m_Springs[i];
      pSpring->UpdateForce();
      pSpring->GetLeftLink()->m_ExtForce += pSpring->GetForceOnLeft();
      pSpring->GetRightLink()->m_ExtForce += pSpring->GetForceOnRight();
    }

	//--- States
	iCount = m_States.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pState = m_States[i];

      pState->ResetConstraintImpulse();
    }


	//--- Systems
	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++) 
    {
      pSystem = m_Systems[i];

      // Union-Find Reset
      pSystem->UF_Reset();
    }
}

inline void srSpace::__DYN_Before_srDYN()
{
	int i, iCount;
	SE3 T;

	// Update External force
	// Control input, Passive element response

	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      m_Systems[i]->DYN_ForwardDynamics_Set00(m_Timestep_dyn_fixed);
    }
}

inline void srSpace::__DYN_After_srDYN()
{
	int i, iCount;
	srSystem * pSystem;
	srCollision * pCollision;

	// Constraint Dynamics 
	// Link Frame Integration
	iCount = m_Systems.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pSystem = m_Systems[i];

      if (pSystem->m_bExcited)
        pSystem->DYN_ImpulseDynamics_Set00(m_FPS_dyn_fixed);

      pSystem->DYN_IntegratePosition_Set00(m_Timestep_dyn_fixed);
      pSystem->UpdateCOMZMP(m_Gravity);
    }

	// Collision Frame Update
	iCount = m_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pCollision = m_Collisions[i];
      pCollision->UpdateFrame();
    }
}

inline void	srSpace::_Step_Simulation_Time()
{
	m_Simulation_Time += m_Timestep_dyn_fixed;
}

inline void srSpace::__DYN_Reset_Post()
{
	int i, iCount;
	srLink * pLink;

	//--- Links
	iCount = m_Links.get_size();
	for (i = 0 ; i < iCount ; i++)
    {
      pLink = m_Links[i];
      pLink->ResetUserExternalForce();
    }
}

void srSpace::_USER_CONTROL()
{
	(*m_pfn_USER_CONTROL)();
}

void srSpace::_USER_CONTROL(void* _pt)
{
	(*m_pfn_USER_CONTROL_2)(_pt);
}

void _do_nothing()
{
}


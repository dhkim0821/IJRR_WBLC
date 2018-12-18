#include "srDyn/srLink.h"
#include "srDyn/srJoint.h"
#include "srDyn/srState.h"
#include "srDyn/srSystem.h"

#include "srDyn/srRevoluteJoint.h"
#include "srDyn/srPrismaticJoint.h"
#include "srDyn/srUniversalJoint.h"


#include "srDyn/srDYN.h"

#define DEGREE_TO_RADIAN							SR_RADIAN;
#define RADIAN_TO_DEGREE							SR_DEGREE;

//**********************************************************************//
// srDYN

srDYN::srDYN()
{
	m_bALLOCATED = false;
}

srDYN::~srDYN()
{
	if (m_bALLOCATED)
	{
		int i, iCount;

		iCount = m_Baked_ContactConstraints.get_size();
		for (i = 0 ; i <  iCount ; i++)
		{
			m_Baked_ContactConstraints[i]._FREE_pContactPts();
		}
	}
}

//======== MARK9 ========//
void srDYN::PRESTEP_MARK9(SystemPtrArray & _systemptrarray, LinkPtrArray & _linkptrarray, JointPtrArray & _jointptrarray, sr_real & _timestep, sr_real & _fps)
{
	_PRESTEP_Set_SpaceVariables___________________MARK9(_timestep, _fps);

	_PRESTEP_TEMP_BuildMassPair___________________MARK9(_linkptrarray);
	_PRESTEP_TEMP_OnOff_MassPair__________________MARK7();
	_PRESTEP_TEMP_Eliminate_SelfCollision_________MARK7();
	
	_PRESTEP_Build_Baked_ContactConstraint________MARK7();

	_PRESTEP_Build_Baked_JointConstraint__________MARK11(_jointptrarray);

	_PRESTEP_Build_Baked_ClosedLoop_______________MARK7(_systemptrarray);

	_PRESTEP_Initialize_Static_Variables__________MARK11();
}

void srDYN::_PRESTEP_Set_SpaceVariables___________________MARK9(sr_real & _timestep, sr_real & _fps)
{
	// fps
	m_Timestep_fixed = _timestep;
	m_FPS_fixed = _fps;


	// PGS option
	m_PGSOption.SetDefault();

	// etc
	ContactConstraint::SetErp(DEFAULT_ERP_CONTACT_srDYN);
	ContactConstraint::SetAllowedPenetration(DEFAULT_ALLOWED_PENETRATION_srDYN);
	ContactConstraint::SetBouncingThreshold(DEFAULT_BOUNCING_THRESHOLD_CONTACT_srDYN);
	ContactConstraint::SetMaximumErpVelocity(DEFAULT_MAXIMUM_ER_VELOCITY_CONTACT_srDYN);
	ContactConstraint::SetMaximumBouncingVelocity(DEFAULT_MAXIMUM_BOUNCING_VELOCITY_CONTACT_srDYN);

	JointConstraint::SetErp(DEFAULT_ERP_JOINTPOSITIONLIMIT_srDYN);
	JointConstraint::SetAllowedPenetration(DEFAULT_ALLOWED_JOINTPOSITIONERROR_srDYN);
	JointConstraint::SetBouncingThreshold(DEFAULT_BOUNCING_THRESHOLD_JOINTPOSITION_srDYN);
	JointConstraint::SetMaximumErpVelocity(DEFAULT_MAXIMUM_ER_VELOCITY_JOINTPOSITION_srDYN);
	JointConstraint::SetMaximumBouncingVelocity(DEFAULT_MAXIMUM_BOUNCING_VELOCITY_JOINTPOSITION_srDYN);

	ClosedLoop::SetErp(DEFAULT_ERP_CLOSEDLOOP_srDYN);
	ClosedLoop::SetAllowedPenetration(DEFAULT_ALLOWED_CLOSEDLOOPERROR_srDYN);
	ClosedLoop::SetMaximumErpVelocity(DEFAULT_MAXIMUM_ER_VELOCITY_CLOSEDLOOP_srDYN);
}



void srDYN::_PRESTEP_TEMP_BuildMassPair___________________MARK9(LinkPtrArray & _linkptrarray)
{
	int i,j,Count;
	MassPair masspair;

	m_MassPairs.clear();
	Count = _linkptrarray.get_size();
	for (i = 0 ; i < Count ; i++)
	{
		masspair.pLeftMass = _linkptrarray[i];
		j = i;
		for ( j++; j < Count ; j++ )
		{
			masspair.pRightMass = _linkptrarray[j];
			m_MassPairs.add_tail(masspair);
		}
	}
}

void srDYN::_PRESTEP_TEMP_OnOff_MassPair__________________MARK7(bool onoff /*= true*/)
{
	int i,iCount;
	
	iCount = m_MassPairs.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		m_MassPairs[i].bCollidable = onoff;
	}
}

void srDYN::_PRESTEP_TEMP_Eliminate_SelfCollision_________MARK7()
{
	int i,iCount;
	MassPair *pMassPair;
	srSystem  *pSystem;

	iCount = m_MassPairs.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pMassPair = &m_MassPairs[i];

		if (pMassPair->pLeftMass->m_pSystem == pMassPair->pRightMass->m_pSystem)
		{
			pSystem = pMassPair->pLeftMass->m_pSystem;

			if (pSystem->m_IsSelfCollision == false)
			{
				pMassPair->bCollidable = false;
			}
		}
	}
}

void srDYN::_PRESTEP_Build_Baked_ContactConstraint________MARK7()
{
	int i, iCount;
	ContactConstraint * pContactConstraint;

	if (m_bALLOCATED)
	{
		int i, iCount;

		iCount = m_Baked_ContactConstraints.get_size();
		for (i = 0 ; i <  iCount ; i++)
		{
			m_Baked_ContactConstraints[i]._FREE_pContactPts();
		}
	}

	m_Baked_ContactConstraints.clear();

	iCount = m_MassPairs.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		__PRESTEP_MassPair_To_ContactConstraint_______MARK7(&m_MassPairs[i]);
	}

	m_CollisionPairArray.clear();

	iCount = m_Baked_ContactConstraints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pContactConstraint = &m_Baked_ContactConstraints[i];
		pContactConstraint->_ALLOC_pContactPts();
		
		__PRESTEP_Build_CollisionPairs________________MARK9(pContactConstraint);
	}
	m_bALLOCATED = true;
}

void srDYN::__PRESTEP_MassPair_To_ContactConstraint_______MARK7(MassPair * pMassPair)
{
	ContactConstraint cc;

	//-- Collision tag check
	if (!pMassPair->bCollidable)
		return;

	//-- Possibility of dynamic response
	// If none of both rigid bodies are dynamic , return
	if (!( pMassPair->pLeftMass->m_DynType == srLink::DYNAMIC ) &&
		!( pMassPair->pRightMass->m_DynType == srLink::DYNAMIC ) )
		return;

	//-- Collision object check
	// If there's no collision object, return
	if ((pMassPair->pLeftMass->m_Collisions.get_size() == 0) ||
		(pMassPair->pRightMass->m_Collisions.get_size() == 0) )
		return;

	//-- Initial self collision check
	// Initially self collided mass pair rejection
#ifdef	INITIAL_SELFCOLLISIONPAIR_EJECTION
	if (pMassPair->pLeftMass->m_pSystem == pMassPair->pRightMass->m_pSystem)
	{
		cc._ALLOC_pContactPts();

		cc.PRESTEP_Set_MassPair_______________________MARK9(pMassPair);
		if (cc.PRESTEP_Test_Collision_____________________MARK7())
		{
			cc._FREE_pContactPts();
			return;
		}
		cc._FREE_pContactPts();
	}
#endif

	// Add to Baked ContactConstraint Array and build it
	int nSize = m_Baked_ContactConstraints.get_size();
	m_Baked_ContactConstraints.set_size(nSize+1,true);
	m_Baked_ContactConstraints[nSize].PRESTEP_Set_MassPair_______________________MARK9(pMassPair);
}

void srDYN::__PRESTEP_Build_CollisionPairs________________MARK9(ContactConstraint * pContactConstraint)
{
	int j,k;
	int jCount,kCount;

	CollisionPair collpair;


	jCount = pContactConstraint->pLeftMass->m_Collisions.get_size();
	kCount = pContactConstraint->pRightMass->m_Collisions.get_size();

	collpair.pContactConstraint = pContactConstraint;
	for (j = 0 ; j < jCount ; j++)
	{
		collpair.pLeftCol =  pContactConstraint->pLeftMass->m_Collisions[j];

		for (k = 0 ; k < kCount ; k++)
		{
			collpair.pRightCol =  pContactConstraint->pRightMass->m_Collisions[k];

			if (collpair.PRESTEP_Find_RoughCheck__Algorithm())
				if (collpair.PRESTEP_Find_NarrowPhase_Algorithm())
					m_CollisionPairArray.add_tail(collpair);
		}
	}
}

void srDYN::_PRESTEP_Build_Baked_JointConstraint__________MARK11(JointPtrArray & _jointptrarray)
{
	int i,iCount;
	
	m_Baked_JointConstraints.clear();

	iCount = _jointptrarray.get_size();	
	for (i = 0 ; i < iCount ; i++)
	{
		__PRESTEP_JointState_To_JointConstraint_______MARK11(_jointptrarray[i]);
	}
}

void srDYN::__PRESTEP_JointState_To_JointConstraint_______MARK11(srJoint * pJoint)
{
	JointConstraint jc;
	
	jc.pJoint = pJoint;
	jc.pSystem = pJoint->m_pSystem;
	jc.actuationtype = pJoint->m_ActType;
	jc.jointtype = pJoint->m_JointType;


	jc.bActive = false;
	jc.lifetime = 0;
	jc.lambda = 0.0;

	srRevoluteJoint * pRJoint;
	srPrismaticJoint * pPJoint;
	srUniversalJoint * pUJoint;
	//XBallJoint		* pBallJoint;
	
	switch (jc.jointtype)
	{
	case srJoint::REVOLUTE:
		jc.m_pRstate = (srRevoluteState*)(pJoint->GetStatePtr());

		pRJoint = (srRevoluteJoint*)pJoint;

		// R-Joint Position Limit Constraint
		if (pRJoint->m_IsPosLimited)
		{
			jc.Limit[0] = pRJoint->m_PosLimit[0] * DEGREE_TO_RADIAN;
			jc.Limit[1] = pRJoint->m_PosLimit[1] * DEGREE_TO_RADIAN;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_R_PositionLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_PositionLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_R;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_R;
			jc.m_pfn_unexcite = &JointConstraint::_unexcite_R;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_R;

			m_Baked_JointConstraints.add_tail(jc);
		}

		// R-Joint Torque Limit Constraint
		if (jc.actuationtype == srJoint::VELOCITY)
		{
			jc.ForceLimit[0] = pRJoint->m_TorqueLimit[0]*m_Timestep_fixed;
			jc.ForceLimit[1] = pRJoint->m_TorqueLimit[1]*m_Timestep_fixed;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_R_TorqueLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_TorqueLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_R;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_R;
			jc.m_pfn_unexcite = &JointConstraint::_unexcite_R;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_R;

			m_Baked_JointConstraints.add_tail(jc);
		}


		break;
	case srJoint::PRISMATIC:
		jc.m_pPstate = (srPrismaticState*)(pJoint->GetStatePtr());

		pPJoint = (srPrismaticJoint*)pJoint;

		// P-Joint Position Limit Constraint
		if (pPJoint->m_IsPosLimited)
		{
			jc.Limit[0] = pPJoint->m_PosLimit[0] * DEGREE_TO_RADIAN;
			jc.Limit[1] = pPJoint->m_PosLimit[1] * DEGREE_TO_RADIAN;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_P_PositionLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_PositionLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_P;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_P;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_P;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_P;

			m_Baked_JointConstraints.add_tail(jc);
		}

		// P-Joint Torque Limit Constraint
		if (jc.actuationtype == srJoint::VELOCITY)
		{
			jc.ForceLimit[0] = pPJoint->m_TorqueLimit[0]*m_Timestep_fixed;
			jc.ForceLimit[1] = pPJoint->m_TorqueLimit[1]*m_Timestep_fixed;


			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_P_TorqueLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_TorqueLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_P;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_P;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_P;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_P;


			m_Baked_JointConstraints.add_tail(jc);
		}

		break;
	case srJoint::UNIVERSAL:
		jc.m_pUstate = (srUniversalState*)(pJoint->GetStatePtr());

		pUJoint = (srUniversalJoint*)pJoint;

		// U-Joint First Position Limit Constraint
		if (pUJoint->m_IsPosLimited[0])
		{
			jc.Limit[0] = pUJoint->m_PosLimit_1[0] * DEGREE_TO_RADIAN;
			jc.Limit[1] = pUJoint->m_PosLimit_1[1] * DEGREE_TO_RADIAN;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_U1_PositionLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_PositionLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_U_1;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_U_1;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_U_1;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_U_1;

			m_Baked_JointConstraints.add_tail(jc);
		}

		// U-Joint Second Position Limit Constraint
		if (pUJoint->m_IsPosLimited[1])
		{
			jc.Limit[0] = pUJoint->m_PosLimit_2[0] * DEGREE_TO_RADIAN;
			jc.Limit[1] = pUJoint->m_PosLimit_2[1] * DEGREE_TO_RADIAN;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_U2_PositionLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_PositionLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_U_2;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_U_2;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_U_2;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_U_2;

			m_Baked_JointConstraints.add_tail(jc);
		}

		// U-Joint Torque Limit Constraint
		if (jc.actuationtype == srJoint::VELOCITY)
		{
			jc.ForceLimit[0] = pUJoint->m_TorqueLimit_1[0]*m_Timestep_fixed;
			jc.ForceLimit[1] = pUJoint->m_TorqueLimit_1[1]*m_Timestep_fixed;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_U1_TorqueLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_TorqueLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_U_1;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_U_1;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_U_1;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_U_1;

			m_Baked_JointConstraints.add_tail(jc);


			jc.ForceLimit[0] = pUJoint->m_TorqueLimit_2[0]*m_Timestep_fixed;
			jc.ForceLimit[1] = pUJoint->m_TorqueLimit_2[1]*m_Timestep_fixed;

			jc.m_pfn_inspect_jointstate = &JointConstraint::_inspect_U2_TorqueLimit;
			jc.m_pfn_getInformation = &JointConstraint::_getInformation_TorqueLimit;
			jc.m_pfn_applyimpulse = &JointConstraint::_applyimpulse_U_2;
			jc.m_pfn_getdelvelocity = &JointConstraint::_getdelvelocity_U_2;
			jc.m_pfn_unexcite= &JointConstraint::_unexcite_U_2;
			jc.m_pfn_setimpulse = &JointConstraint::_setimpulse_U_2;

			m_Baked_JointConstraints.add_tail(jc);
		}
		

		break;
	default:
		break;
	}
}

void srDYN::_PRESTEP_Build_Baked_ClosedLoop_______________MARK7(SystemPtrArray & _systemptrarray)
{
	int i,iCount;
	int j,jCount;

	ClosedLoop closedloop;
	
	m_Baked_ClosedLoops.clear();

	iCount = _systemptrarray.get_size();	
	for (i = 0 ; i < iCount ; i++)
	{
		jCount = _systemptrarray[i]->m_linkpairs_for_closedloop.get_size();
		for (j = 0 ; j < jCount ; j++)
		{
			srSystem::linkpair_for_closedloop & tmp = _systemptrarray[i]->m_linkpairs_for_closedloop[j];

			closedloop.pSystem = _systemptrarray[i];
			closedloop.pLeftMass = tmp.LeftLink;
			closedloop.pRightMass = tmp.RightLink;
			
			closedloop.RelativeFrame = tmp.RelativeFrame;

			m_Baked_ClosedLoops.add_tail(closedloop);
		}
	}
}

void srDYN::_PRESTEP_Initialize_Static_Variables__________MARK11()
{
	int i,iCount;
	srSystem * pSystem;

	//==== Gather Possible ISLAND IDs ====//
	m_Possible_ISLAND_IDs.clear();
	//--- Solid Constraints
	//- Closed Loop
	iCount = m_Baked_ClosedLoops.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSystem = m_Baked_ClosedLoops[i].UF_Find_Constraint();
		m_Possible_ISLAND_IDs.check_add_tail(pSystem);
	}

	//--- Blink Constraints
	//- Contact
	iCount = m_Baked_ContactConstraints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSystem = m_Baked_ContactConstraints[i].UF_Find_Constraint();
		m_Possible_ISLAND_IDs.check_add_tail(pSystem);
	}
	//- Joint Constraint
	iCount = m_Baked_JointConstraints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSystem = m_Baked_JointConstraints[i].UF_Find_Constraint();
		m_Possible_ISLAND_IDs.check_add_tail(pSystem);
	}
	m_nPossible_ISLAND_IDs = m_Possible_ISLAND_IDs.get_size();


	//==== Initialize Static Variables ====//
	//--- Solid Constraints
	Solid_Constraints.clear();
	//- Closed Loop
	iCount = m_Baked_ClosedLoops.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		Solid_Constraints.add_tail(&m_Baked_ClosedLoops[i]);
	}
	nSolid_Constraints = Solid_Constraints.get_size();

	//--- Blink Constraints
	Blink_Constraints.clear();
	nPossible_Blink_Constraints = 0;
	//- Contact
	nPossible_Blink_Constraints += m_Baked_ContactConstraints.get_size();
	//- Joint Constraint Limit
	nPossible_Blink_Constraints += m_Baked_JointConstraints.get_size();
	Blink_Constraints.set_size(nPossible_Blink_Constraints);
	nBlink_Constraints = 0;

	//--- All Constraints
	nPossible_All_Constraints = nSolid_Constraints + nPossible_Blink_Constraints;


	//--- Island
	m_islands.clear();
	m_islands.set_size(m_nPossible_ISLAND_IDs);
	m_nislands = 0;
	iCount = m_islands.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		m_islands[i].m_AllConstraints.clear();
		m_islands[i].m_AllConstraints.set_size(nPossible_All_Constraints);
		m_islands[i].m_nConstraints = 0;

		m_islands[i].m_cinfo.recip_timestep = m_FPS_fixed;
	}
}

void srDYN::RUNTIME_MARK9()
{
	_RUNTIME_Detect_Blink_Constraints_MARK10();
	_RUNTIME_Build_ISLANDs_MARK10();
	_RUNTIME_Solve_Constraints_by_ISLANDs_MARK9();
}

inline void srDYN::_RUNTIME_Detect_Blink_Constraints_MARK10()
{
	int i,iCount1,iCount2;

	nBlink_Constraints = 0;

	//--- Contact Constraint Detection
	iCount1 = m_Baked_ContactConstraints.get_size();
	iCount2 = m_CollisionPairArray.get_size();

	for (i = 0 ; i < iCount1 ; i++)
	{
		ContactConstraint & contactconstraint = m_Baked_ContactConstraints[i];


		// Refreshing
		// : Delete invalid contact points.
		//   And deactivate ContactConstraint and contact points.
		contactconstraint.Filter_InvalidContactPoints_And_Refresh____MARK7();
	}

	for (i = 0 ; i < iCount2 ; i++)
	{
		CollisionPair & collpair = m_CollisionPairArray[i];

		// Collision Detection (CollisionPair Loop)
		// : Detect Geometrical contact points.
		//   And activate ContactConstraint and contact points.
		if( collpair.RoughCheck____________________________________MARK7() )
		{
			collpair.NarrowPhase_Algorithm_________________________MARK7();
		}
	}

	for (i = 0 ; i < iCount1 ; i++)
	{
		ContactConstraint & contactconstraint = m_Baked_ContactConstraints[i];

		// Filtering
		// : Gather active ContacConstraints and Prepare for solver.
		//	 And delete all inactive contact points.
		if (contactconstraint.bActive)
		{
			// Delete inactive contact points
			contactconstraint.Filter_InactiveContactPoints_______________MARK7(); 

			// Prepare for solver
			contactconstraint.Prepare_for_Solver_________________________MARK7();

			Blink_Constraints[nBlink_Constraints++] = &contactconstraint;
		}
		else // contactconstraint.bActive == false // 
		{
			contactconstraint.nContactPts = 0; // Delete all contact points;
		}
	}

	//--- Joint Limit Detection
	iCount1 = m_Baked_JointConstraints.get_size();
	for (i = 0 ; i < iCount1 ; i++)
	{
		JointConstraint & jc = m_Baked_JointConstraints[i];

		if ( jc.Inspect_JointState() )
		{
			Blink_Constraints[nBlink_Constraints++] = &jc;
		}
	}
}

inline void srDYN::_RUNTIME_Build_ISLANDs_MARK10()
{
	int i, j;
	srSystem * pSystem;
	srSystem * ID;
	bool exist;
	Constraint * pConstraint;

	//--- Set ISLAND ID of all Possible_ISLAND_IDs
	// Sorting
	m_nislands = 0;
	for (i = 0 ; i < m_nPossible_ISLAND_IDs ; i++) 
	{
		pSystem = m_Possible_ISLAND_IDs[i];
		ID = pSystem->m_UF_id = Constraint::UF_Find_System(pSystem);

		exist = false;

		// Existing ISLAND
		for (j = 0 ; j < m_nislands ; j++)
		{
			if (m_islands[j].m_ID == ID)
			{
				exist = true;
				break;
			}
		}
		if (exist) continue;

		// New ISLAND
		ISLAND & island = m_islands[m_nislands];
		island.m_ID = ID;
		island.m_nConstraints = 0;
		ID->m_UF_idx = m_nislands;
		m_nislands++;
	}


	//--- Create islands
	//-- Solid Constraint
	for (i = 0 ; i < nSolid_Constraints ; i++)
	{
		pConstraint = Solid_Constraints[i];
		ID = pConstraint->UF_Find_Constraint();

		ISLAND & island = m_islands[ID->m_UF_idx];

		island.m_AllConstraints[island.m_nConstraints++] = pConstraint;
	}

	//-- Blink Constraint
	for (i = 0 ; i < nBlink_Constraints ; i++)
	{
		pConstraint = Blink_Constraints[i];
		ID = pConstraint->UF_Find_Constraint();

		ISLAND & island = m_islands[ID->m_UF_idx];

		island.m_AllConstraints[island.m_nConstraints++] = pConstraint;
	}
}

inline void srDYN::_RUNTIME_Build_ISLANDs_MARK9()
{
	int i, j;
	srSystem * pSystem;
	srSystem * ID;
	bool exist;
	Constraint * pConstraint;

	//--- Set ISLAND ID of all Possible_ISLAND_IDs
	for (i = 0 ; i < m_nPossible_ISLAND_IDs ; i++) 
	{
		pSystem = m_Possible_ISLAND_IDs[i];
		pSystem->m_UF_id = Constraint::UF_Find_System(pSystem);
	}


	//--- Create islands
	m_nislands = 0;

	//-- Solid Constraint
	for (i = 0 ; i < nSolid_Constraints ; i++)
	{
		pConstraint = Solid_Constraints[i];
		ID = pConstraint->UF_Find_Constraint();

		exist = false;

		// Existing ISLAND
		for (j = 0 ; j < m_nislands ; j++)
		{
			ISLAND & island = m_islands[j];
			if (island.m_ID == ID)
			{
				exist = true;
				island.m_AllConstraints[island.m_nConstraints++] = pConstraint;
				break;
			}
		}
		if (exist) continue;

		// New ISLAND
		ISLAND & island = m_islands[m_nislands++];
		island.m_ID = ID;
		island.m_AllConstraints[0] = pConstraint;
		island.m_nConstraints = 1;
	}

	//-- Blink Constraint
	for (i = 0 ; i < nBlink_Constraints ; i++)
	{
		pConstraint = Blink_Constraints[i];
		ID = pConstraint->UF_Find_Constraint();

		exist = false;

		// Existing ISLAND
		for (j = 0 ; j < m_nislands ; j++)
		{
			ISLAND & island = m_islands[j];
			if (island.m_ID == ID)
			{
				exist = true;
				island.m_AllConstraints[island.m_nConstraints++] = pConstraint;
				break;
			}
		}
		if (exist) continue;

		// New ISLAND
		ISLAND & island = m_islands[m_nislands++];
		island.m_ID = ID;
		island.m_AllConstraints[0] = pConstraint;
		island.m_nConstraints = 1;
	}
}

inline void srDYN::_RUNTIME_Solve_Constraints_by_ISLANDs_MARK9()
{
	int i;
	for (i = 0 ; i < m_nislands ; i++)
	{
		m_islands[i]._PGS_Solve_Constraint_____________________MARK7(&m_PGSOption);
	}
}

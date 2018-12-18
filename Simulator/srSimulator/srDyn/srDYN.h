#pragma once

#include "srDyn/srObject.h"

#include "srDyn/srConstraint.h"
#include "srDyn/srContactConstraint.h"
#include "srDyn/srJointConstraint.h"
#include "srDyn/srClosedLoopConstraint.h"
#include "srDyn/srISLAND.h"

#include "LieGroup/_array.h"

//***********************************************//
// Compile Option

//--- Routine Option
#define	INITIAL_SELFCOLLISIONPAIR_EJECTION			1

//-- Default variables
#define DEFAULT_ERP_CONTACT_srDYN							0.8
#define DEFAULT_ALLOWED_PENETRATION_srDYN					0.01
#define DEFAULT_BOUNCING_THRESHOLD_CONTACT_srDYN			0.2
#define DEFAULT_MAXIMUM_ER_VELOCITY_CONTACT_srDYN			10//10
#define DEFAULT_MAXIMUM_BOUNCING_VELOCITY_CONTACT_srDYN		30//30

#define DEFAULT_ERP_JOINTPOSITIONLIMIT_srDYN					0.8
#define DEFAULT_ALLOWED_JOINTPOSITIONERROR_srDYN				0.0001
#define DEFAULT_BOUNCING_THRESHOLD_JOINTPOSITION_srDYN			0.2
#define DEFAULT_MAXIMUM_ER_VELOCITY_JOINTPOSITION_srDYN			10
#define DEFAULT_MAXIMUM_BOUNCING_VELOCITY_JOINTPOSITION_srDYN	20

#define DEFAULT_ERP_CLOSEDLOOP_srDYN							0.8
#define DEFAULT_ALLOWED_CLOSEDLOOPERROR_srDYN					0.01
#define DEFAULT_MAXIMUM_ER_VELOCITY_CLOSEDLOOP_srDYN			10



//**********************************************************************//
// Type definition
typedef _array<srSystem*>			SystemPtrArray;
typedef	_array<MassPair>			MassPairArray;
typedef _array<CollisionPair>		CollisionPairArray;
typedef	_array<ContactConstraint>	ContactConstraintArray;
typedef _array<ClosedLoop>			ClosedLoopArray;
typedef _array<JointConstraint>		JointConstraintArray;
typedef _array<ISLAND>				ISLANDArray;

typedef _array<srLink*>				LinkPtrArray;
typedef _array<srState*>			StatePtrArray;
typedef _array<srJoint*>			JointPtrArray;

//**********************************************************************//


//**********************************************************************//
// srDYN
class srDYN
{
 public:
  srDYN();
  ~srDYN();

  //////////////////////////////////////////////////////////////////////////
  //======= MARK9 =======//
  MassPairArray		m_MassPairs;	// temporary..

  //Etc
  bool								m_bALLOCATED;
  CollisionPairArray					m_CollisionPairArray;

  // Space variables
  sr_real								m_Timestep_fixed;
  sr_real								m_FPS_fixed; // frame per second  ( : 1/timestep )

  //--- Constraint Objects
  //- Solid Constraints
  ClosedLoopArray						m_Baked_ClosedLoops;		// closed loop constraint objects
  //- Blink Constraints
  ContactConstraintArray				m_Baked_ContactConstraints;	// contact constraint objects
  JointConstraintArray				m_Baked_JointConstraints;	// joint constraint objects

  //--- Possible Island ID
  SystemPtrArray						m_Possible_ISLAND_IDs;
  int									m_nPossible_ISLAND_IDs;

  //--- Solver Option
  PGSOption							m_PGSOption;

  //=== RUNTIME ===//
  //--- Constraint Ptrs
  ConstraintPtrArray					Solid_Constraints;
  int									nSolid_Constraints;

  ConstraintPtrArray					Blink_Constraints;
  int									nPossible_Blink_Constraints;
  int									nBlink_Constraints;

  int									nPossible_All_Constraints;

  //--- Islands
  ISLANDArray							m_islands;
  int									m_nislands;



  void	PRESTEP_MARK9(SystemPtrArray & _systemptrarray, LinkPtrArray & _linkptrarray, JointPtrArray & _jointptrarray, sr_real & _timestep, sr_real & _fps);

  void	_PRESTEP_Set_SpaceVariables___________________MARK9(sr_real & _timestep, sr_real & _fps);
  void	_PRESTEP_TEMP_BuildMassPair___________________MARK9(LinkPtrArray & _linkptrarray);
  void	_PRESTEP_TEMP_OnOff_MassPair__________________MARK7(bool onoff = true);
  void	_PRESTEP_TEMP_Eliminate_SelfCollision_________MARK7();

  void	_PRESTEP_Build_Baked_ContactConstraint________MARK7();
  void	__PRESTEP_MassPair_To_ContactConstraint_______MARK7(MassPair * pMassPair);
  void	__PRESTEP_Build_CollisionPairs________________MARK9(ContactConstraint * pContactConstraint);

  void	_PRESTEP_Build_Baked_JointConstraint__________MARK11(JointPtrArray & _jointptrarray);	// OLD_1
  void	__PRESTEP_JointState_To_JointConstraint_______MARK11(srJoint * pJoint);					// OLD_1
	
  void	_PRESTEP_Build_Baked_ClosedLoop_______________MARK7(SystemPtrArray & _systemptrarray);

  void	_PRESTEP_Initialize_Static_Variables__________MARK11();


  void	RUNTIME_MARK9();
  void	_RUNTIME_Detect_Blink_Constraints_MARK10();
  void	_RUNTIME_Build_ISLANDs_MARK9();
  void	_RUNTIME_Build_ISLANDs_MARK10();
  void	_RUNTIME_Solve_Constraints_by_ISLANDs_MARK9(); // need multi-threading
};
//**********************************************************************//

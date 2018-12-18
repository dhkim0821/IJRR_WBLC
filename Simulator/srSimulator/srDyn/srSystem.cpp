#include "srDyn/srSystem.h"

srSystem::srSystem()
{
	m_BaseLink = NULL;
	m_BaseLinkType = DYNAMIC;
	m_IsSelfCollision = false;
	m_linkpairs_for_closedloop.clear();
	m_COM = Vec3(0.0);
	m_ZMP = Vec3(0.0);
}

void srSystem::SetBaseLink(srLink* v)
{
	m_BaseLink = v;
}

void srSystem::SetBaseLinkType(BASELINKTYPE v)
{
	m_BaseLinkType = v;
}

void srSystem::SetSelfCollision(bool v /* = true */)
{
	m_IsSelfCollision = v;
}

srLink* srSystem::GetBaseLink()
{
	return m_BaseLink;
}

srSystem::BASELINKTYPE srSystem::GetBaseLinkType()
{
	return m_BaseLinkType;
}

bool srSystem::IsSelfCollision()
{
	return m_IsSelfCollision;
}

void srSystem::BackupInitState()
{
	m_BaseLink->BackupInitState();

	for(int i = 0 ; i < m_KIN_Joints.get_size() ; ++i)
		m_KIN_Joints[i]->GetStatePtr()->BackupInitState();
}

void srSystem::RestoreInitState()
{
	m_BaseLink->RestoreInitState();

	for(int i = 0 ; i < m_KIN_Joints.get_size() ; ++i)
		m_KIN_Joints[i]->GetStatePtr()->RestoreInitState();
}

void srSystem::KIN_ValidateSystem()
{
	m_KIN_Links.clear();
	m_KIN_Joints.clear();
	m_KIN_Collisions.clear();
	m_KIN_Sensors.clear();

	int i, j, nLinkStack;
	srLink * pLink;
	srLink * pCLink;
	srJoint * pJoint;
	srCollision * pCollision;
	srSensor * pSensor;
	_array<srLink*> LinkStack;


	pLink = m_BaseLink;
	pLink->m_IsBaseLink = true;
	pLink->m_pSystem = this;
	pLink->m_ParentLink = NULL;
	pLink->m_ParentJoint = NULL;

	m_KIN_Links.add_tail(pLink);

	// Collision & Sensor Entities
	for (i = 0 ; i < pLink->m_Collisions.get_size() ; i++)
	{
		pCollision = pLink->m_Collisions[i];
		m_KIN_Collisions.add_tail(pCollision);
	}
	for (i = 0 ; i < pLink->m_Sensors.get_size() ; i++)
	{
		pSensor = pLink->m_Sensors[i];
		m_KIN_Sensors.add_tail(pSensor);
	}


	LinkStack.clear();
	nLinkStack = 0;

	goto Here_I_am;
	while (nLinkStack > 0)
	{
		nLinkStack--;
		pLink = LinkStack[nLinkStack];
		pLink->m_IsBaseLink = false;
		pLink->m_pSystem = this;

		LinkStack.pop(nLinkStack);

		m_KIN_Links.add_tail(pLink);

		// Collision & Sensor Entity
		for (i = 0 ; i < pLink->m_Collisions.get_size() ; i++)
		{
			pCollision = pLink->m_Collisions[i];
			m_KIN_Collisions.add_tail(pCollision);
		}
		for (i = 0 ; i < pLink->m_Sensors.get_size() ; i++)
		{
			pSensor = pLink->m_Sensors[i];
			m_KIN_Sensors.add_tail(pSensor);
		}

Here_I_am:
		pLink->m_ChildLinks.clear();
		for (j = 0 ; j < pLink->m_ChildJoints.get_size() ; j++)
		{
			pJoint = pLink->m_ChildJoints[j];
			pJoint->m_pSystem = this;

			m_KIN_Joints.add_tail(pJoint);
			pJoint->Set_UpdateFrame_Ftn();

			if (pJoint->m_ChildLink != NULL)
			{
				pCLink = pJoint->m_ChildLink;

				LinkStack.add_tail(pCLink);
				nLinkStack++;

				pJoint->m_FS_Force_SE3 = pJoint->m_ChildLinkToJoint;

				pLink->m_ChildLinks.add_tail(pCLink);
				pCLink->m_ParentLink = pLink;
				pCLink->m_ParentJoint = pJoint;
			}
		}
	}

	KIN_Initialize();
}

void srSystem::KIN_Initialize()
{
	int i, iCount;
	srLink * pLink;
	srJoint * pJoint;

	// Home 자세 & 조인트 screw 업데이트
	// m_KIN_Links[0] is base link
	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++) // Link[0] : base link
	{
		pLink = m_KIN_Links[i];
		pJoint = pLink->m_ParentJoint;

		pLink->m_M = pJoint->m_ParentLinkToJoint / pJoint->m_ChildLinkToJoint;

		if (pJoint->m_PriorityIndex == 0) // straight
		{
			pJoint->FS_SetScrew(0);
		}
		else //if (pJoint->m_PriorityIndex == 1) // reversed
		{
			pJoint->FS_SetScrew(1);
		}
	}

	// Multi-body 와 sole-body 구분
	if (m_KIN_Links.get_size() > 1) // multi-body system
	{
		m_IsLonelySystem = false;
	}
	else // lonely system
	{
		m_IsLonelySystem = true;
	}
}

void srSystem::KIN_UpdateFrame_All_The_Entity()
{
	int i, iCount;
	srLink		* pLink;
	srCollision * pCollision;
	srSensor	* pSensor;
	srJoint		* pJoint;

	// Link
	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i <  iCount ; i++)
	{
		pLink = m_KIN_Links[i];
		pLink->m_Frame = pLink->m_ParentLink->m_Frame * pLink->FS_Transform();
	}

	// Collision
	iCount = m_KIN_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pCollision = m_KIN_Collisions[i];
		pCollision->UpdateFrame();
	}

	//// Sensor
	iCount = m_KIN_Sensors.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSensor = m_KIN_Sensors[i];
		pSensor->UpdateFrame();
	}

	// Joint
	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pJoint = m_KIN_Joints[i];
		pJoint->UpdateFrame();
	}
}

void srSystem::KIN_UpdateFrame_All_The_Entity_Light()
{
	int i, iCount;
	srLink *	pLink;
	srCollision * pCollision;
	srSensor	* pSensor;
	srJoint	* pJoint;

	// Link
	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i <  iCount ; i++)
	{
		pLink = m_KIN_Links[i];
		pLink->m_Frame = pLink->m_ParentLink->m_Frame * pLink->FS_GetSE3();
	}

	// Collision
	iCount = m_KIN_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pCollision = m_KIN_Collisions[i];
		pCollision->UpdateFrame();
	}

	// Sensor
	iCount = m_KIN_Sensors.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSensor = m_KIN_Sensors[i];
		pSensor->UpdateFrame();
	}

	// Joint
	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pJoint = m_KIN_Joints[i];
		pJoint->UpdateFrame();
	}
}

void srSystem::KIN_LinkFramePropagation()
{
	int i, iCount;
	srLink *	pLink;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i <  iCount ; i++)
	{
		pLink = m_KIN_Links[i];
		pLink->m_Frame = pLink->m_ParentLink->m_Frame * pLink->FS_Transform();
	}
}

void srSystem::KIN_UpdateFrame_CollsionEntity()
{
	int i, iCount;
	srCollision * pCollision;

	iCount = m_KIN_Collisions.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pCollision = m_KIN_Collisions[i];
		pCollision->UpdateFrame();
	}
}

void srSystem::KIN_UpdateFrame_SensorEntity()
{
	// Open when entities are valid.
	int i, iCount;
	srSensor * pSensor;

	iCount = m_KIN_Sensors.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pSensor = m_KIN_Sensors[i];
		pSensor->UpdateFrame();
	}
}

void srSystem::KIN_UpdateFrame_JointEntity()
{
	int i, iCount;
	srJoint * pJoint;

	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pJoint = m_KIN_Joints[i];
		pJoint->UpdateFrame();
	}
}

void srSystem::UpdateCOMZMP(Vec3& g)
{
	int i, iCount;
	srLink* pLink;

	sr_real totalMass = 0.0;	// total mass
	sr_real totalMoment = 0.0;	// total moment
	Vec3 mc(0.0);	// total mass * mass center position
	Vec3 ma(0.0);	// total mass * mass center acceleration

	sr_real lm;	// each link mass
	Vec3 lmc;	// each link mass center
	Vec3 lma;	// each link acceleration of mass center

	iCount = m_KIN_Links.get_size();
	for(i = 0 ; i < iCount ; ++i)
	{
		pLink = m_KIN_Links[i];
		lm  = pLink->GetMass();
		lmc = pLink->GetMassCenter();
		lma = pLink->GetLinearAcc(pLink->GetOffset()) - g;	// take account gravity.

		// total mass
		totalMass += lm;
		// total mass * mass center position
		mc += lm * lmc;
		// total moment
		totalMoment += lm * lma[2];
		// total moment
		ma[0] += lm * (lma[2]*lmc[0] - lma[0]*lmc[2]);
		ma[1] += lm * (lma[2]*lmc[1] - lma[1]*lmc[2]);
	}

	if(totalMass > 0.0)
		m_COM = mc / totalMass;
	if(totalMoment != 0.0)
		m_ZMP = ma / totalMoment;
}

Vec3& srSystem::GetCOM()
{
	return m_COM;
}

Vec3& srSystem::GetZMP()
{
	return m_ZMP;
}
//////////////////////////////////////////////////////////////////////////
// DIFFKIN

void srSystem::DIFFKIN_Initialize()
{
	if (m_IsLonelySystem)	// lonely system
	{
		m_pfn_diffkin_integrateposition = &srSystem::__L_IntegratePosition_Euler;
		m_pfn_diffkin_linkvelocitypropagation = &srSystem::__L_LinkVelocityPropagation;
	}
	else					// multi-body system
	{
		m_pfn_diffkin_integrateposition = &srSystem::__FS_IntegratePosition_Euler;
		m_pfn_diffkin_linkvelocitypropagation = &srSystem::__FS_LinkVelocityPropagation;
	}
}

void srSystem::DIFFKIN_LinkVelocityPropagation()
{
	(this->*m_pfn_diffkin_linkvelocitypropagation)();
}

void srSystem::__FS_LinkVelocityPropagation()
{
	int i, iCount;
	srLink * pMass;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
			+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
	}
}

void srSystem::__L_LinkVelocityPropagation()
{
	// do nothing
}

void srSystem::DIFFKIN_IntegratePosition(sr_real & _step)
{
	(this->*m_pfn_diffkin_integrateposition)(_step);
}

void srSystem::__L_IntegratePosition_Euler(sr_real & _step)
{
	if (m_BaseLinkType != FIXED)
		m_L_Link->m_Frame *= Exp(_step * m_L_Link->m_Vel);
}

void srSystem::__FS_IntegratePosition_Euler(sr_real & _step)
{
	int i, iCount;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType != FIXED)
		pMass->m_Frame *= Exp(_step * pMass->m_Vel);

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->UpdatePosition(_step);

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Frame = pMass->m_ParentLink->m_Frame * pMass->FS_Transform();
	}
}

//////////////////////////////////////////////////////////////////////////
// DYN

void srSystem::DYN_Initialize()
{
	int i, iCount;
	srLink * pLink;
	srJoint * pJoint;
	srState * pState;

	//--- Link
	// : Set dynamics Inertia, dyntype
	// : Set Velocity, Acceleration And External Force Zero
	iCount = m_KIN_Links.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pLink = m_KIN_Links[i];
		pLink->SetDynType();

		pLink->m_Vel = 0.0;
		pLink->m_Acc = 0.0;
		pLink->m_ExtForce = 0.0;
	}

	//--- Joints(Actuator)
	//--- States
	// : Set Actuator's Ftn Ptr
	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pJoint = m_KIN_Joints[i];
		pJoint->Initialize();
		pJoint->SetDeviceOnOff(pJoint->IsDeviceOn());
		pJoint->m_FS_Force = 0.0;

		pState = pJoint->GetStatePtr();
		pState->ResetVel();
		pState->ResetAcc();
		pState->ResetCommand();
	}

	UnExciteSystem();
	UF_Reset();

	DIFFKIN_Initialize();

	if (m_IsLonelySystem)	// lonely system
	{
		m_L_Link = m_KIN_Links[0];
		m_L_Link_Inertia = m_L_Link->m_Inertia;
		m_L_Link_InvInertia = Inv(m_L_Link->m_Inertia);

		m_pfn_dyn_forwarddynamics_set00 = &srSystem::_L_ForwardDynamics_Set00;
		m_pfn_dyn_impulsedynamics_set00 = &srSystem::_L_Impulsedynamics_set00;
		m_pfn_dyn_integrateposition_set00 = &srSystem::_L_IntegratePosition_Set00;
	}
	else					// multi-body system
	{
		m_pfn_dyn_forwarddynamics_set00 = &srSystem::_FS_ForwardDynamics_Set00;
		m_pfn_dyn_impulsedynamics_set00 = &srSystem::_FS_Impulsedynamics_set00;
		m_pfn_dyn_integrateposition_set00 = &srSystem::_FS_IntegratePosition_Set00;
	}
}

void srSystem::DYN_ForwardDynamics_Set00(sr_real & _step)
{
	(this->*m_pfn_dyn_forwarddynamics_set00)(_step);
}

void srSystem::_FS_ForwardDynamics_Set00(sr_real & _step)
{
	//__FS_UpdateArticulatedInertia();
	//__FS_UpdateBiasforce();
	//__FS_UpdateAcceleration();
	//__FS_IntegrateVelocity_Euler(_step);

	// Articulated Inertia
	int i,j, nCount, iCount;
	srLink * pMass;
	srLink * pChildMass;
	AInertia AIjari;
	dse3 Cias;
	se3	DV;

	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i--)
	{
		pMass = m_KIN_Links[i];
		pMass->m_AInertia = pMass->m_Inertia;

		nCount = pMass->m_ChildLinks.get_size();
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateAIS_K(AIjari);
			pMass->m_AInertia.AddTransform(AIjari,Inv(pChildMass->FS_GetSE3()));
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_AInertia = pMass->m_Inertia;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateAIS_K(AIjari);
			pMass->m_AInertia.AddTransform(AIjari,Inv(pChildMass->FS_GetSE3()));
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateAIS_K();
		}
	}

	// BiasForce
	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i-- )
	{
		pMass = m_KIN_Links[i];

		pMass->m_Bias.dad(-pMass->m_Vel, pMass->m_Inertia * pMass->m_Vel);
		pMass->m_Bias -= pMass->m_ExtForce;

		nCount = pMass->m_ChildLinks.get_size();
		for (j = 0; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateBiasforce(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_Bias.dad(-pMass->m_Vel, pMass->m_Inertia * pMass->m_Vel);
		pMass->m_Bias -= pMass->m_ExtForce;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasforce(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasforce(Cias);
		}
	}
	// Acceleration
	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_Acc = pMass->m_AInertia % -pMass->m_Bias;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount; i++)
	{
		pMass = m_KIN_Links[i];
		DV.InvAd(pMass->FS_GetSE3(),pMass->m_ParentLink->m_Acc);
		pMass->FS_UpdateAcc(DV);

		pMass->FS_UpdateForce_Link();
	}

	// Integrate Velocity
	pMass = m_KIN_Links[0];
	if (m_BaseLinkType != FIXED)
		pMass->m_Vel += _step * pMass->m_Acc;

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
	{
		m_KIN_Joints[i]->GetStatePtr()->UpdateVelocity(_step);
	}

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
			+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
	}
}

void srSystem::_L_ForwardDynamics_Set00(sr_real & _step)
{
	//__L_UpdateBiasforce();
	//__L_UpdateAcceleration();
	//__L_IntegrateVelocity_Euler(_step);

	if (m_BaseLinkType == DYNAMIC)
	{
		m_L_Link->m_Bias.dad(m_L_Link->m_Vel, m_L_Link->m_Inertia * m_L_Link->m_Vel);
		m_L_Link->m_Bias += m_L_Link->m_ExtForce;

		//m_L_Link->m_Acc = m_L_Link_InvInertia * m_L_Link->m_Bias;
		m_L_Link->m_Acc = m_L_Link_Inertia % m_L_Link->m_Bias;
	}

	if (m_BaseLinkType != FIXED)
	{
		m_L_Link->m_Vel += _step * m_L_Link->m_Acc;
	}
}

void srSystem::__FS_UpdateArticulatedInertia()
{
	int i,j, nCount;
	srLink * pMass;
	srLink * pChildMass;
	AInertia AIjari;

	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i--)
	{
		pMass = m_KIN_Links[i];
		pMass->m_AInertia = pMass->m_Inertia;

		nCount = pMass->m_ChildLinks.get_size();
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateAIS_K(AIjari);
			pMass->m_AInertia.AddTransform(AIjari,Inv(pChildMass->FS_GetSE3()));
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_AInertia = pMass->m_Inertia;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateAIS_K(AIjari);
			pMass->m_AInertia.AddTransform(AIjari,Inv(pChildMass->FS_GetSE3()));
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateAIS_K();
		}
	}
}

void srSystem::__FS_UpdateBiasforce()
{
	int i, j, nCount;
	srLink * pMass;
	srLink * pChildMass;

	dse3 Cias;

	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i-- )
	{
		pMass = m_KIN_Links[i];

		pMass->m_Bias.dad(-pMass->m_Vel, pMass->m_Inertia * pMass->m_Vel);
		pMass->m_Bias -= pMass->m_ExtForce;

		nCount = pMass->m_ChildLinks.get_size();
		for (j = 0; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateBiasforce(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_Bias.dad(-pMass->m_Vel, pMass->m_Inertia * pMass->m_Vel);
		pMass->m_Bias -= pMass->m_ExtForce;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasforce(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasforce(Cias);
		}
	}
}

void srSystem::__FS_UpdateAcceleration()
{
	int i, iCount;
	se3	DV;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_Acc = pMass->m_AInertia % -pMass->m_Bias;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount; i++)
	{
		pMass = m_KIN_Links[i];
		DV.InvAd(pMass->FS_GetSE3(),pMass->m_ParentLink->m_Acc);
		pMass->FS_UpdateAcc(DV);

		pMass->FS_UpdateForce_Link();
	}
}

void srSystem::__FS_IntegrateVelocity_Euler(sr_real & _step)
{
	int i, iCount;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType != FIXED)
		pMass->m_Vel += _step * pMass->m_Acc;

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
	{
		m_KIN_Joints[i]->GetStatePtr()->UpdateVelocity(_step);
	}

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
			+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
	}
}

void srSystem::__L_UpdateBiasforce()
{
	if (m_BaseLinkType == DYNAMIC)
	{
		m_L_Link->m_Bias.dad(m_L_Link->m_Vel, m_L_Link->m_Inertia * m_L_Link->m_Vel);
		m_L_Link->m_Bias += m_L_Link->m_ExtForce;
	}
}

void srSystem::__L_UpdateAcceleration()
{
	if (m_BaseLinkType == DYNAMIC)
	{
		//m_L_Link->m_Acc = m_L_Link_InvInertia * m_L_Link->m_FS_Bias;
		m_L_Link->m_Acc = m_L_Link_Inertia % m_L_Link->m_Bias;
	}
}

void srSystem::__L_IntegrateVelocity_Euler(sr_real & _step)
{
	if (m_BaseLinkType != FIXED)
		m_L_Link->m_Vel += _step * m_L_Link->m_Acc;
}

void srSystem::DYN_ImpulseDynamics_Set00(sr_real & _fps)
{
	(this->*m_pfn_dyn_impulsedynamics_set00)(_fps);
}

void srSystem::_FS_Impulsedynamics_set00(sr_real & _fps)
{
	//_FS_UpdateBiasImpulse();
	//_FS_UpdateDelVelocity();
	//_FS_UpdateVelocity_plus_DelVelocity();
	//_FS_UpdateAcceleration_with_DelVelocity(_fps);
	//UnExciteSystem();

	int i, j, nCount, iCount;
	srLink * pMass;
	srLink * pChildMass;
	srState * pState;

	se3	DV;
	dse3 Cias;

	// UpdateBiasImpulse
	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i--)
	{
		pMass = m_KIN_Links[i];

		pMass->m_Bias = -pMass->m_ConstraintImpulse;

		nCount = pMass->m_ChildLinks.get_size();
		for ( j = 0; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateBiasImp(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_Bias = -pMass->m_ConstraintImpulse;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasImp(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasImp(Cias);
		}
	}

	// UpdateDelVelocity UpdateForce
	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_DelVel = pMass->m_AInertia % -pMass->m_Bias;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		DV.InvAd(pMass->FS_GetSE3(),pMass->m_ParentLink->m_DelVel);
		pMass->FS_UpdateDelVel(DV);

		pMass->FS_UpdateImpulse_Link(_fps);
	}

	// Update Vel, Acc
	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_Vel += pMass->m_DelVel;
		pMass->m_Acc += (_fps*pMass->m_DelVel);
	}

	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pState = m_KIN_Joints[i]->GetStatePtr();

		pState->UpdateVelWithDelVel();
		pState->UpdateAccWithDelVel(_fps);
		pState->UpdateTorqueWithImp(_fps);
	}

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];

		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
					+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
		//pMass->m_sVel += pMass->m_DelVel;

		pMass->m_Acc += (_fps*pMass->m_DelVel);
	}

	// UnExcite
	m_bExcited = false;
}

void srSystem::_L_Impulsedynamics_set00(sr_real & _fps)
{
	//L_UpdateDelVelocity();
	//L_IntegrateVelocity_plus_DelVelocity();
	//L_UpdateAcceleration_with_DelVelocity(_fps);
	//UnExciteSystem();

	if (m_BaseLinkType == DYNAMIC)
	{
		//m_L_Link->m_DelVel = m_L_Link_InvInertia * m_L_Link->m_ConstraintImp;
		m_L_Link->m_DelVel = m_L_Link_Inertia % m_L_Link->m_ConstraintImpulse;
		m_L_Link->m_Vel += m_L_Link->m_DelVel;
		m_L_Link->m_Acc += (_fps * m_L_Link->m_DelVel);
		m_bExcited = false;
	}
}

void srSystem::__FS_UpdateBiasImpulse()
{
	int i, j, nCount;
	srLink * pMass;
	srLink * pChildMass;

	dse3 Cias;
	for (i = m_KIN_Links.get_size() - 1 ; i >= 1 ; i--)
	{
		pMass = m_KIN_Links[i];

		pMass->m_Bias = -pMass->m_ConstraintImpulse;

		nCount = pMass->m_ChildLinks.get_size();
		for ( j = 0; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];

			pChildMass->FS_UpdateBiasImp(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}

	pMass = m_KIN_Links[0]; // baselink
	nCount = pMass->m_ChildLinks.get_size();

	if (m_BaseLinkType == DYNAMIC)
	{
		pMass->m_Bias = -pMass->m_ConstraintImpulse;
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasImp(Cias);
			pMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
		}
	}
	else
	{
		for (j = 0 ; j < nCount ; j++)
		{
			pChildMass = pMass->m_ChildLinks[j];
			pChildMass->FS_UpdateBiasImp(Cias);
		}
	}
}

void srSystem::__FS_UpdateDelVelocity(sr_real & _fps)
{
	int i, iCount;
	se3	DV;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_DelVel = pMass->m_AInertia % -pMass->m_Bias;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		DV.InvAd(pMass->FS_GetSE3(),pMass->m_ParentLink->m_DelVel);
		pMass->FS_UpdateDelVel(DV);

		pMass->FS_UpdateImpulse_Link(_fps);
	}
}

void srSystem::__FS_UpdateVelocity_with_DelVelocity()
{
	int i, iCount;
	srLink * pMass;
	srState * pState;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_Vel += pMass->m_DelVel;

	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pState = m_KIN_Joints[i]->GetStatePtr();

		pState->UpdateVelWithDelVel();
	}

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
			+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
	}
}

void srSystem::__FS_UpdateAcceleration_with_DelVelocity(sr_real & _fps)
{
	int i, iCount;
	srLink * pMass;
	srState * pState;


	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_Acc += (_fps*pMass->m_DelVel);

	iCount = m_KIN_Joints.get_size();
	for (i = 0 ; i < iCount ; i++)
	{
		pState = m_KIN_Joints[i]->GetStatePtr();

		pState->UpdateAccWithDelVel(_fps);
		pState->UpdateTorqueWithImp(_fps);
	}


	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Acc += (_fps*pMass->m_DelVel);
	}
}

void srSystem::__L_UpdateDelVelocity()
{
	if (m_BaseLinkType == DYNAMIC)
		//m_L_Link->m_DelVel = m_L_Link_InvInertia * m_L_Link->m_ConstraintImpulse;
		m_L_Link->m_DelVel = m_L_Link_Inertia % m_L_Link->m_ConstraintImpulse;
}

void srSystem::__L_UpdateVelocity_with_DelVelocity()
{
	if (m_BaseLinkType == DYNAMIC)
		m_L_Link->m_Vel += m_L_Link->m_DelVel;
}

void srSystem::__L_UpdateAcceleration_with_DelVelocity(sr_real & _fps)
{
	if (m_BaseLinkType == DYNAMIC)
		m_L_Link->m_Acc += (_fps * m_L_Link->m_DelVel);
}

void srSystem::DYN_IntegratePosition_Set00(sr_real & _step)
{
	(this->*m_pfn_dyn_integrateposition_set00)(_step);
}

void srSystem::_FS_IntegratePosition_Set00(sr_real & _step)
{
	//DIFFKIN_IntegratePosition(_step);
	//DIFFKIN_LinkVelocityPropagation();

	//__FS_IntegratePosition_Euler(_step);
	//__FS_LinkVelocityPropagation();

	int i, iCount;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType != FIXED)
		pMass->m_Frame *= Exp(_step * pMass->m_Vel);

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->UpdatePosition(_step);

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Frame = pMass->m_ParentLink->m_Frame * pMass->FS_Transform();

		pMass->m_Vel = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel)
			+ pMass->m_ParentJoint->FS_UpdateLocalVelocity();
	}
}

void srSystem::_L_IntegratePosition_Set00(sr_real & _step)
{
	//DIFFKIN_IntegratePosition(_step);
	//DIFFKIN_LinkVelocityPropagation();

	//__L_IntegratePosition_Euler(step);
	//__L_LinkVelocityPropagation();

	if (m_BaseLinkType != FIXED)
		m_L_Link->m_Frame *= Exp(_step * m_L_Link->m_Vel);
}


//-- Union-Find
void srSystem::UF_Reset() // inline 해야되는데
{
	m_UF_id = this;
	m_UF_sz = 1;
	//m_UF_idx = -1;
}

//-- On Impulse Test
void srSystem::ExciteSystem()
{
	m_bExcited = true;
}

void srSystem::UnExciteSystem()
{
	m_bExcited = false;
}

//-- Impulse Test
void srSystem::FS_Reset_Bias_T()
{
	int i, iCount;
	srLink * pMass;

	m_KIN_Links[0]->m_Bias = 0.0;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];

		pMass->m_Bias = 0.0;
		pMass->m_ParentJoint->FS_ResetT();
	}
}

void srSystem::FS_UpdateBiasImpulse(srLink * pMass, const dse3& imp)
{
	// XLink * pMass : 포인터 인수를 사용하여, 지역변수를 하나 줄일 수 있다.
	srLink * pCurrentMass;
	srLink * pChildMass;
	dse3	Cias;

	pCurrentMass = pMass;

	pCurrentMass->m_Bias -= imp;
	//pCurrentMass->m_FS_Bias = -imp;

	while(m_KIN_Links[0] != pCurrentMass)
	{
		pChildMass = pCurrentMass;
		pCurrentMass = pCurrentMass->m_ParentLink;

		pChildMass->FS_UpdateBiasImp(Cias);

		pCurrentMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
	}
}

void srSystem::FS_UpdateBiasImpulse(srLink * pMass)
{
	srLink * pCurrentMass;
	srLink * pChildMass;
	dse3	Cias;

	pCurrentMass = pMass;

	while(m_KIN_Links[0] != pCurrentMass)
	{
		pChildMass = pCurrentMass;
		pCurrentMass = pCurrentMass->m_ParentLink;

		pChildMass->FS_UpdateBiasImp(Cias);

		pCurrentMass->m_Bias += InvdAd(pChildMass->FS_GetSE3(), Cias);
	}
}

void srSystem::FS_UpdateDelVelocity()
{
	int i, iCount;
	se3	DV;
	srLink * pMass;

	pMass = m_KIN_Links[0];
	if (m_BaseLinkType == DYNAMIC)
		pMass->m_DelVel = pMass->m_AInertia % -pMass->m_Bias;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		DV.InvAd(pMass->FS_GetSE3(),pMass->m_ParentLink->m_DelVel);
		pMass->FS_UpdateDelVel(DV);
	}
}

void srSystem::L_UpdateDelVelocity(const dse3& imp)
{
	if (m_BaseLinkType == DYNAMIC)
		//m_L_Link->m_DelVel = m_L_Link_InvInertia * imp;
		m_L_Link->m_DelVel = m_L_Link_Inertia % imp;
}

//--- Not used right now
//-- ConstraintForce Reset
void srSystem::FS_Reset_ConstraintImpulse()
{
	int i, iCount;

	iCount = m_KIN_Links.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Links[i]->m_ConstraintImpulse = 0.0;

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->ResetConstraintImpulse();
}

void srSystem::L_Reset_ConstraintImpulse()
{
	m_L_Link->m_ConstraintImpulse = 0.0;
}
//-- Error Correction
void srSystem::ResetPosErrVelocity()
{
	int i, iCount;

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->ResetPosErrVel();

	iCount = m_KIN_Links.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Links[i]->m_Vel_PosErr = 0.0;
}

void srSystem::FS_MassPosErrVelocityPropagation()
{
	int i, iCount;
	srLink * pMass;

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel_PosErr = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel_PosErr)
			+ pMass->m_ParentJoint->FS_UpdatePosErrorLocalVelocity();
	}
}

void srSystem::FS_IntegratePosErrVelocity_plus_DelVelocity()
{
	int i, iCount;
	srLink * pMass;

	if (m_BaseLinkType == DYNAMIC)
		m_KIN_Links[0]->m_Vel_PosErr += m_KIN_Links[0]->m_DelVel;

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->UpdatePosErrVelWithDelVel();

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Vel_PosErr = InvAd(pMass->FS_GetSE3(), pMass->m_ParentLink->m_Vel_PosErr)
			+ pMass->m_ParentJoint->FS_UpdatePosErrorLocalVelocity();
	}
}

void srSystem::FS_IntegratePosition_PosErrVel_Euler(sr_real & _step)
{
	int i, iCount;
	srLink * pMass;

	if (m_BaseLinkType == DYNAMIC)
		m_KIN_Links[0]->m_Frame *= Exp(_step * m_KIN_Links[0]->m_Vel_PosErr);

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->UpdatePosWithPosErrVel(_step);

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Frame = pMass->m_ParentLink->m_Frame * pMass->FS_Transform();
	}
}

void srSystem::FS_IntegratePosition_PosErrVel_plus_Velocity_Euler(sr_real & _step)
{
	int i, iCount;
	srLink * pMass;

	if (m_BaseLinkType == DYNAMIC)
		m_KIN_Links[0]->m_Frame *= Exp(_step * (m_KIN_Links[0]->m_Vel + m_KIN_Links[0]->m_Vel_PosErr));

	iCount = m_KIN_Joints.get_size();
	for(i = 0 ; i < iCount ; i++)
		m_KIN_Joints[i]->GetStatePtr()->UpdatePosWithCorrection(_step);

	iCount = m_KIN_Links.get_size();
	for (i = 1 ; i < iCount ; i++)
	{
		pMass = m_KIN_Links[i];
		pMass->m_Frame = pMass->m_ParentLink->m_Frame * pMass->FS_Transform();
	}
}


void srSystem::MakeClosedLoop(srLink * pLeftLink, srLink * pRightLink, SE3 RelativeFrame)
{
	linkpair_for_closedloop tmp;
	
	tmp.LeftLink = pLeftLink;
	tmp.RightLink = pRightLink;
	tmp.RelativeFrame = RelativeFrame;

	m_linkpairs_for_closedloop.add_tail(tmp);
}

//
//////////////////////////////////////////////////////////////////////////

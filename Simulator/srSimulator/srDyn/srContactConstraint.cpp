#include "srDyn/srContactConstraint.h"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "srDyn/srLink.h"

#define SQRT2_OVER_2								0.7071067811865475244008443621048490

//**********************************************************************//
// Collision Pair

bool CollisionPair::RoughCheck____________________________________MARK7()
{
	return (this->*m_pfn_RoughCheck_Algoritm)();
}

int	CollisionPair::NarrowPhase_Algorithm_________________________MARK7()
{
	return (this->*m_pfn_NarrowPhase_Algoritm)();
}

bool CollisionPair::PRESTEP_Find_RoughCheck__Algorithm()
{
	srGeometryInfo::SHAPETYPE LeftType = pLeftCol->GetCollisionShape();
	srGeometryInfo::SHAPETYPE RightType = pRightCol->GetCollisionShape();

	if (LeftType == srGeometryInfo::PLANE)
	{
		m_pfn_RoughCheck_Algoritm = &CollisionPair::_PlaneOthers________RoughCheck_TEMP;
		return true;
	}
	else if (RightType == srGeometryInfo::PLANE)
	{
		m_pfn_RoughCheck_Algoritm = &CollisionPair::_OthersPlane________RoughCheck_TEMP;
		return true;
	}
	else
	{
		m_pfn_RoughCheck_Algoritm = &CollisionPair::_OthersOthers_______RoughCheck_TEMP;
		return true;
	}
	return false;
}

bool CollisionPair::PRESTEP_Find_NarrowPhase_Algorithm()
{
	srGeometryInfo::SHAPETYPE LeftType = pLeftCol->GetCollisionShape();
	srGeometryInfo::SHAPETYPE RightType = pRightCol->GetCollisionShape();

	switch(LeftType)
	{
	case srGeometryInfo::BOX:

		switch(RightType)
		{
		case srGeometryInfo::BOX:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxBox_____________MARK8;
			return true;

			break;
		case srGeometryInfo::SPHERE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxSphere__________MARK8;
			return true;

			break;
		case srGeometryInfo::CYLINDER:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxCylinder________MARK8;
			return true;

			break;
		case srGeometryInfo::CAPSULE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxCapsule_________MARK8;
			return true;

			break;
		case srGeometryInfo::PLANE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxPlane___________MARK8;
			return true;

			break;
		default:
			return false;

			break;
		}

		break;
	case srGeometryInfo::SPHERE:

		switch(RightType)
		{
		case srGeometryInfo::BOX:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereBox__________MARK8;
			return true;

			break;
		case srGeometryInfo::SPHERE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereSphere_______MARK8;
			return true;

			break;
		case srGeometryInfo::CYLINDER:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereCylinder_____MARK8;
			return true;

			break;
		case srGeometryInfo::CAPSULE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereCapsule______MARK8;
			return true;

			break;
		case srGeometryInfo::PLANE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SpherePlane________MARK8;
			return true;

			break;
		default:
			return false;

			break;
		}

		break;
	case srGeometryInfo::CYLINDER:

		switch(RightType)
		{
		case srGeometryInfo::BOX:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderBox________MARK8;
			return true;

			break;
		case srGeometryInfo::SPHERE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderSphere_____MARK8;
			return true;

			break;
		case srGeometryInfo::CYLINDER:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderCylinder___MARK8;
			return true;

			break;
		case srGeometryInfo::CAPSULE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderCapsule____MARK8;
			return true;

			break;
		case srGeometryInfo::PLANE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderPlane______MARK8;
			return true;

			break;
		default:
			return false;

			break;
		}

		break;
	case srGeometryInfo::CAPSULE:

		switch(RightType)
		{
		case srGeometryInfo::BOX:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleBox_________MARK8;
			return true;

			break;
		case srGeometryInfo::SPHERE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleSphere______MARK8;
			return true;

			break;
		case srGeometryInfo::CYLINDER:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleCylinder____MARK8;
			return true;

			break;
		case srGeometryInfo::CAPSULE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleCapsule_____MARK8;
			return true;

			break;
		case srGeometryInfo::PLANE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsulePlane_______MARK8;
			return true;

			break;
		default:
			return false;

			break;
		}

		break;

	case srGeometryInfo::PLANE:

		switch(RightType)
		{
		case srGeometryInfo::BOX:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneBox___________MARK8;
			return true;

			break;
		case srGeometryInfo::SPHERE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneSphere________MARK8;
			return true;

			break;
		case srGeometryInfo::CYLINDER:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneCylinder______MARK8;
			return true;

			break;
		case srGeometryInfo::CAPSULE:
			m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneCapsule_______MARK8;
			return true;

			break;
		case srGeometryInfo::PLANE:
			return false;


			break;
		default:
			return false;

			break;
		}

		break;

	default:
		return false;

		break;
	}
	return false;
}


//**********************************************************************//


//**********************************************************************//
// Contact Constraint

sr_real ContactConstraint::erp_contact = 0.8;
sr_real ContactConstraint::allowedpenetration = 0.01;
sr_real ContactConstraint::bouncing_threshold = 0.2;
sr_real ContactConstraint::maximum_erp_velocity = 10;
sr_real ContactConstraint::maximum_bouncing_velocity = 30;


void ContactConstraint::SetErp(sr_real _erp)
{
	erp_contact = _erp;
}

void ContactConstraint::SetAllowedPenetration(sr_real _allowedpenetration)
{
	allowedpenetration = _allowedpenetration;
}

void ContactConstraint::SetBouncingThreshold(sr_real _bouncingthreshold)
{
	bouncing_threshold = _bouncingthreshold;
}

void ContactConstraint::SetMaximumErpVelocity(sr_real _maximum_erp_velocity)
{
	maximum_erp_velocity = _maximum_erp_velocity;
}

void ContactConstraint::SetMaximumBouncingVelocity(sr_real _maximum_bouncing_velocity)
{
	maximum_bouncing_velocity = _maximum_bouncing_velocity;
}

bool ContactConstraint::PRESTEP_Test_Collision_____________________MARK7()
{
	int j,k,jCount,kCount;

	CollisionPair collpair;

	jCount = pLeftMass->m_Collisions.get_size();
	kCount = pRightMass->m_Collisions.get_size();

	collpair.pContactConstraint = this;
	for (j = 0 ; j < jCount ; j++)
	{
		collpair.pLeftCol = pLeftMass->m_Collisions[j];

		for (k = 0 ; k < kCount ; k++)
		{
			collpair.pRightCol =  pRightMass->m_Collisions[k];

			if (collpair.PRESTEP_Find_NarrowPhase_Algorithm())
				if ( 0 < collpair.NarrowPhase_Algorithm_________________________MARK7() )
					return true;
		}
	}

	return false;
}

void	ContactConstraint::PRESTEP_Set_MassPair_______________________MARK9(MassPair* pmasspair)
{
	// Set MassPair
	pLeftMass = pmasspair->pLeftMass;
	pRightMass = pmasspair->pRightMass;

	// Set SystemPtr
	pLSystem = pLeftMass->m_pSystem;
	pRSystem = pRightMass->m_pSystem;

	// Set Material Information
	rFriction_coeff = _PRESTEP_Calculate_FrictionCoeff___________MARK7(pLeftMass->m_Friction, pRightMass->m_Friction);
	if (rFriction_coeff < FRICTIONCOEFF_EPS)
		bFriction = false;
	else
		bFriction = true;

	rRestitution_coeff = _PRESTEP_Calculate_RestitutionCoeff________MARK7(pLeftMass->m_Restitution, pRightMass->m_Restitution);
	if (rRestitution_coeff < RESTITUTIONCOEFF_EPS)
		bBounce = false;
	else
		bBounce = true;


	// Initialize contact information
	bActive = false;
	nContactPts = 0;

	// Set Virtual Function
	_PRESTEP_Set_Virual_Function_______________MARK7();
}

void	ContactConstraint::_ALLOC_pContactPts()
{
	int i;
	for(i = 0 ; i < MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR ; i++)
	{
		pContactPts[i] = new Contact;
	}
}

void	ContactConstraint::_FREE_pContactPts()
{
	int i;
	for(i = 0 ; i < MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR ; i++)
	{
		delete pContactPts[i];
	}
}

sr_real	ContactConstraint::_PRESTEP_Calculate_FrictionCoeff___________MARK7(sr_real leftmat, sr_real rightmat)
{
	return min(leftmat, rightmat);
}

sr_real	ContactConstraint::_PRESTEP_Calculate_RestitutionCoeff________MARK7(sr_real leftmat, sr_real rightmat)
{
	//return min(leftmat, rightmat);
	return leftmat * rightmat;
}

void		ContactConstraint::Filter_InvalidContactPoints_And_Refresh____MARK7()
{
	bActive = false;

	Vec3 gpoint_Left, gpoint_Right, projected;
	sr_real dist;
	for (int i = nContactPts-1 ; i >= 0 ; i--)
	{
		Contact & cp = *pContactPts[i];

		gpoint_Left = pLeftMass->GetFrame() * cp.localpoint_Left;
		gpoint_Right = pRightMass->GetFrame() * cp.localpoint_Right;
		dist = Inner((gpoint_Left - gpoint_Right),cp.NormalVec);


		//cp.bValid = false;

		if (dist > CONTACT_THRESHOLD)
		{
			RemoveContactPoint(i);
		}
		else
		{
			projected = gpoint_Left - dist * cp.NormalVec;
			projected = gpoint_Right - projected;
			dist = SquareSum(projected);
			if (dist > CONTACT_THRESHOLD_SQR)
			{
				RemoveContactPoint(i);
			}
			else
			{
				cp.bActive = false;
			}
		}
	}
}

void		ContactConstraint::Filter_InactiveContactPoints_______________MARK7()
{
	for (int i = nContactPts-1 ; i >= 0 ; i--)
	{
		Contact & cp = *pContactPts[i];

		if (!cp.bActive)
		{
			RemoveContactPoint(i);
		}
	}
}

void		ContactConstraint::Prepare_for_Solver_________________________MARK7()
{
	// set number of dimension
	// set contact jacobian
	// unite
	int i;
	if (bFriction)
	{
		int tmp = 0;
		Vec3 tmp_tangent1, tmp_tangent2;

		nd = 3 * nContactPts;
		for (i = 0 ; i < nContactPts ; i++)
		{
			Contact & cp = *pContactPts[i];

			TangentPlane(cp.NormalVec,tmp_tangent1,tmp_tangent2);

			JacobianLeft[tmp] = InvdAd(cp.localpoint_Left,InvRotate(pLeftMass->GetFrame(),cp.NormalVec));
			JacobianRight[tmp] = InvdAd(cp.localpoint_Right,InvRotate(pRightMass->GetFrame(),-cp.NormalVec));
			JacobianLeft[tmp+1] = InvdAd(cp.localpoint_Left,InvRotate(pLeftMass->GetFrame(),tmp_tangent1));
			JacobianRight[tmp+1] = InvdAd(cp.localpoint_Right,InvRotate(pRightMass->GetFrame(),-tmp_tangent1));
			JacobianLeft[tmp+2] = InvdAd(cp.localpoint_Left,InvRotate(pLeftMass->GetFrame(),tmp_tangent2));
			JacobianRight[tmp+2] = InvdAd(cp.localpoint_Right,InvRotate(pRightMass->GetFrame(),-tmp_tangent2));

			tmp += 3;
		}
	}
	else
	{
		nd = nContactPts;
		for (i = 0 ; i < nContactPts ; i++)
		{
			Contact & cp = *pContactPts[i];

			JacobianLeft[i] = InvdAd(cp.localpoint_Left,InvRotate(pLeftMass->GetFrame(),cp.NormalVec));
			JacobianRight[i] = InvdAd(cp.localpoint_Right,InvRotate(pRightMass->GetFrame(),-cp.NormalVec));
		}
	}
	UF_Unite();
}

void			ContactConstraint::HowAboutThisPoint(Vec3& point , Vec3& normal , sr_real & penetration)
{
	// CONTACT_REDUCTION_MAXIMIZE_AREA == 1
	// First, Find NearestPoint and if found, Replace it
	// if not, Add it, if already filled, then Sort and Add it.

	Vec3 localpoint_Left = pLeftMass->GetFrame() % point;
	int insertIdx = FindNearestContactPoint(localpoint_Left);
	if (insertIdx >= 0)
	{
		if (!pContactPts[insertIdx]->bActive)
		{
			pContactPts[insertIdx]->lifetime++;
		}

		InsertContactPointAt(point,normal,penetration,localpoint_Left,insertIdx); // Replaced
	}
	else
	{
		if (nContactPts == MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR)
		{
			insertIdx = SortContactPoints(localpoint_Left, penetration);
			if (insertIdx >= 0)
			{
				InsertContactPointAt(point,normal,penetration,localpoint_Left,insertIdx);
				pContactPts[insertIdx]->lifetime = 0; // Sorted
			}
		}
		else // (nContactPts < MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR)
		{
			InsertContactPointAt(point,normal,penetration,localpoint_Left,nContactPts);
			pContactPts[nContactPts++]->lifetime = 0; // Newly Added
		}
	}
}

inline int ContactConstraint::SortContactPoints(Vec3& localpoint_Left, sr_real & penetration)
{
#if MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR == 4
	int maxPenetrationIndex = -1;
	sr_real maxPenetration = penetration;
	for (int i = 0 ; i < 4 ; i++)
	{
		Contact & cp = *pContactPts[i];

		if (cp.Penetration > maxPenetration)
		{
			maxPenetrationIndex = i;
			maxPenetration = cp.Penetration;
		}
	}

	Vec3 v1,v2,v3,v4,v5;
	sr_real d1,d2,d3,d4;
	sr_real maxarea;
	int insertIdx;


	switch(maxPenetrationIndex)
	{
	case 0:
		v1 = pContactPts[1]->localpoint_Left - pContactPts[0]->localpoint_Left;
		v2 = pContactPts[2]->localpoint_Left - pContactPts[0]->localpoint_Left;
		v3 = pContactPts[3]->localpoint_Left - pContactPts[0]->localpoint_Left;

		d1= SquareSum(Cross(v2,v3));
		d2= SquareSum(Cross(v1,v3));
		d3= SquareSum(Cross(v1,v2));

		insertIdx = 1;
		maxarea = d1;
		if (d2> maxarea)
		{
			insertIdx = 2;
			maxarea = d2;
		}
		if (d3> maxarea)
		{
			insertIdx = 3;
		}
		return insertIdx;
	case 1:
		v1= pContactPts[0]->localpoint_Left - pContactPts[1]->localpoint_Left;
		v2= pContactPts[2]->localpoint_Left - pContactPts[1]->localpoint_Left;
		v3= pContactPts[3]->localpoint_Left - pContactPts[1]->localpoint_Left;

		d1= SquareSum(Cross(v2,v3));
		d2= SquareSum(Cross(v1,v3));
		d3= SquareSum(Cross(v1,v2));

		insertIdx = 0;
		maxarea = d1;
		if (d2 > maxarea)
		{
			insertIdx = 2;
			maxarea = d2;
		}
		if (d3 > maxarea)
		{
			insertIdx = 3;
		}
		return insertIdx;
	case 2:
		v1 = pContactPts[0]->localpoint_Left - pContactPts[2]->localpoint_Left;
		v2 = pContactPts[1]->localpoint_Left - pContactPts[2]->localpoint_Left;
		v3 = pContactPts[3]->localpoint_Left - pContactPts[2]->localpoint_Left;

		d1= SquareSum(Cross(v2,v3));
		d2= SquareSum(Cross(v1,v3));
		d3= SquareSum(Cross(v1,v2));

		insertIdx = 0;
		maxarea = d1;
		if (d2> maxarea)
		{
			insertIdx = 1;
			maxarea = d2;
		}
		if (d3> maxarea)
		{
			insertIdx = 3;
		}
		return insertIdx;
	case 3:
		v1= pContactPts[0]->localpoint_Left - pContactPts[3]->localpoint_Left;
		v2= pContactPts[1]->localpoint_Left - pContactPts[3]->localpoint_Left;
		v3= pContactPts[2]->localpoint_Left - pContactPts[3]->localpoint_Left;

		d1 = SquareSum(Cross(v2,v3));
		d2 = SquareSum(Cross(v1,v3));
		d3 = SquareSum(Cross(v1,v2));

		insertIdx = 0;
		maxarea = d1;
		if (d2> maxarea)
		{
			insertIdx = 1;
			maxarea = d2;
		}
		if (d3 > maxarea)
		{
			insertIdx = 2;
		}
		return insertIdx;
	default:
		v1= pContactPts[1]->localpoint_Left - pContactPts[0]->localpoint_Left;
		v2= pContactPts[2]->localpoint_Left - pContactPts[0]->localpoint_Left;
		v3= pContactPts[3]->localpoint_Left - pContactPts[0]->localpoint_Left;
		v4= pContactPts[2]->localpoint_Left - pContactPts[1]->localpoint_Left;
		v5= pContactPts[3]->localpoint_Left - pContactPts[1]->localpoint_Left;


		d1= SquareSum(Cross(v4,v5));
		d2= SquareSum(Cross(v2,v3));
		d3= SquareSum(Cross(v1,v3));
		d4= SquareSum(Cross(v1,v2));

		insertIdx = 0;
		maxarea = d1;
		if (d2 > maxarea)
		{
			insertIdx = 1;
			maxarea = d2;
		}
		if (d3> maxarea)
		{
			insertIdx = 2;
			maxarea = d3;
		}
		if (d4 > maxarea)
		{
			insertIdx = 3;
		}
		return insertIdx;
	}
#else
#if MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR == 3
	Vec3 a10 = pContactPts[1]->localpoint_Left - pContactPts[0]->localpoint_Left;
	Vec3 a20 = pContactPts[2]->localpoint_Left - pContactPts[0]->localpoint_Left;
	Vec3 a21 = pContactPts[2]->localpoint_Left - pContactPts[1]->localpoint_Left;

	Vec3 an0 = localpoint_Left - pContactPts[0]->localpoint_Left;
	Vec3 an1 = localpoint_Left - pContactPts[1]->localpoint_Left;

	sr_real area_n = SquareSum(Cross(a10,a20));
	sr_real area_1 = SquareSum(Cross(an0,a20));
	sr_real area_2 = SquareSum(Cross(an0,a10));
	sr_real area_0 = SquareSum(Cross(a21,an1));


	int insertIdx = -1;
	sr_real maxarea = area_n;
	if (area_1 > maxarea)
	{
		insertIdx = 1;
		maxarea = area_1;
	}
	if (area_2 > maxarea)
	{
		insertIdx = 2;
		maxarea = area_2;
	}
	if (area_0 > maxarea)
	{
		insertIdx = 0;
		maxarea = area_0;
	}
	return insertIdx;
#else
#endif // MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR == 3
#endif // MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR == 4
}

inline int ContactConstraint::FindNearestContactPoint(Vec3& localpoint_Left)
{
	sr_real dist2 = CONTACT_THRESHOLD_SQR;
	sr_real dist1;
	int nearestPointtIndex = -1;
	for (int i = 0 ; i < nContactPts ; i++)
	{
		Contact & cp = *pContactPts[i];

		dist1 = SquareSum(cp.localpoint_Left - localpoint_Left);
		if (dist1 < dist2)
		{
			dist2 = dist1;
			nearestPointtIndex = i;
		}
	}
	return nearestPointtIndex;
}

inline void ContactConstraint::InsertContactPointAt(Vec3 & point, Vec3 & normal, sr_real & penetration, Vec3& localpoint_Left, int insertIdx)
{
	Contact &cp = *pContactPts[insertIdx];

	cp.globalpoint = point;
	cp.NormalVec = normal;
	cp.Penetration = penetration;

	cp.localpoint_Left = localpoint_Left;
	cp.localpoint_Right = pRightMass->GetFrame() % point;

	bActive = true;
	cp.bActive = true;
}

inline void		ContactConstraint::RemoveContactPoint(int _index)
{
	int lastIndex = nContactPts - 1;
	if (_index !=  lastIndex)
	{
		Contact * pTemp = pContactPts[_index];
		pContactPts[_index] = pContactPts[lastIndex];
		pContactPts[lastIndex] = pTemp;
	}
	nContactPts--;
}

inline void		ContactConstraint::TangentPlane(const Vec3& n, Vec3& p, Vec3& q)
{
	if (fabs(n[2]) > SQRT2_OVER_2) {
		// choose p in y-z plane
		sr_real a = n[1]*n[1] + n[2]*n[2];
		sr_real k = 1.0/sqrt(a);
		p.SetValues(0,-n[2]*k,n[1]*k);
		// set q = n x p
		q.SetValues(a*k,-n[0]*p[2],n[0]*p[1]);
	}
	else {
		// choose p in x-y plane
		sr_real a = n[0]*n[0] + n[1]*n[1];
		sr_real k = 1.0/sqrt(a);
		p.SetValues(-n[1]*k,n[0]*k,0);
		// set q = n x p
		q.SetValues(-n[2]*p[1],n[2]*p[0],a*k);
	}
}

void ContactConstraint::_PRESTEP_Set_Virual_Function_______________MARK7()
{
	//-- UF_Unite
	if (pLeftMass->m_DynType == srLink::DYNAMIC && pRightMass->m_DynType == srLink::DYNAMIC)
	{
		if (pLSystem == pRSystem)
		{
			m_pfn_unite = &ContactConstraint::_unite_do_nothing;
		}
		else
		{
			// 양쪽다 Dynamic Mass 면 unite 함수 동작
			m_pfn_unite = &ContactConstraint::_unite_do_something;
		}
	}
	else
	{
		// 한쪽이라도 Dynamic Mass가 아니면 절연
		m_pfn_unite = &ContactConstraint::_unite_do_nothing;
	}

	//-- UF_Find_Contact
	// 절연된 (Dynamic Mass 가 아닌) Mass 의 ID를 return 하면 안된다.
	if (pLeftMass->m_DynType == srLink::DYNAMIC)
	{
		m_pfn_find_constraint = &ContactConstraint::_find_return_leftID;
	}
	else
	{
		m_pfn_find_constraint = &ContactConstraint::_find_return_rightID;
	}


	//-- ApplyImpulse
	if(! (pLeftMass->m_DynType == srLink::DYNAMIC) )			// Only RightMass is Dynamic
	{
		if (pRSystem->m_IsLonelySystem) // Lonely System?
			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_L_Right;
		else
			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_FS_Right;
	}
	else if (! (pRightMass->m_DynType == srLink::DYNAMIC) )	// Only LeftMass is Dynamic
	{
		if (pLSystem->m_IsLonelySystem) // Lonely System?
			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_L_Left;
		else
			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_FS_Left;
	}
	else										// Both RightMass and LefttMass are Dynamic
	{
		if (pLSystem == pRSystem)	// Same System, can not be Lonely
			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_FS_Both;
		else						// Different System
		{
			if (pLSystem->m_IsLonelySystem)	// Left System Lonely?
				m_pfn_applyimpulse_Left = &ContactConstraint::_applyimpulse_L_Left;
			else
				m_pfn_applyimpulse_Left = &ContactConstraint::_applyimpulse_FS_Left;

			if (pRSystem->m_IsLonelySystem)	// Right System Lonely?
				m_pfn_applyimpulse_Right = &ContactConstraint::_applyimpulse_L_Right;
			else
				m_pfn_applyimpulse_Right = &ContactConstraint::_applyimpulse_FS_Right;

			m_pfn_applyimpulse = &ContactConstraint::_applyimpulse_Both;
		}
	}

	//-- GetDelVelocity
	if (! (pLeftMass->m_DynType == srLink::DYNAMIC)	)		// Only RightMass is Dynamic
	{
		m_pfn_getdelvelocity = &ContactConstraint::_getdelvelocity_RightMove;	// Right Dynamic
	}
	else if (! (pRightMass->m_DynType == srLink::DYNAMIC) )	// Only LeftMass is Dynamic
	{
		m_pfn_getdelvelocity = &ContactConstraint::_getdelvelocity_LeftMove;	// Left Dynamic
	}
	else										// Both RightMass and LefttMass are Dynamic
	{
		m_pfn_getdelvelocity = &ContactConstraint::_getdelvelocity_BothMove;	// Both Dynamic
	}

	//-- Excite & UnExcite
	if (! (pLeftMass->m_DynType == srLink::DYNAMIC) )			// Only RightMass is Dynamic
	{
		m_pfn_excite = &ContactConstraint::_excite_RightSystem;
		m_pfn_unexcite = &ContactConstraint::_unexcite_RightSystem;
	}
	else if (! (pRightMass->m_DynType == srLink::DYNAMIC))	// Only LeftMass is Dynamic
	{
		m_pfn_excite = &ContactConstraint::_excite_LeftSystem;
		m_pfn_unexcite = &ContactConstraint::_unexcite_LeftSystem;
	}
	else										// Both RightMass and LefttMass are Dynamic
	{
		if (pLSystem == pRSystem)	// Same System, can not be Lonely
		{
			m_pfn_excite = &ContactConstraint::_excite_LeftSystem;
			m_pfn_unexcite = &ContactConstraint::_unexcite_LeftSystem;
		}
		else						// Different System
		{
			m_pfn_excite = &ContactConstraint::_excite_BothSystem;
			m_pfn_unexcite = &ContactConstraint::_unexcite_BothSystem;
		}
	}

	//-- GetRelVelocity
	if (pLeftMass->m_DynType == srLink::STATIC)			// Only RightMass is Dynamic
	{
		m_pfn_getrelvelocity = &ContactConstraint::_getrelvelocity_RightMove;	// Right Dynamic
	}
	else if (pRightMass->m_DynType == srLink::STATIC)	// Only LeftMass is Dynamic
	{
		m_pfn_getrelvelocity = &ContactConstraint::_getrelvelocity_LeftMove;	// Left Dynamic
	}
	else										// Both RightMass and LefttMass are Dynamic or KINEMATIC
	{
		m_pfn_getrelvelocity = &ContactConstraint::_getrelvelocity_BothMove;	// Both Dynamic or KINEMATIC
	}

}

srSystem* ContactConstraint::_find_return_leftID()
{
	return pLSystem->m_UF_id;
}

srSystem* ContactConstraint::_find_return_rightID()
{
	return pRSystem->m_UF_id;
}

srSystem* ContactConstraint::UF_Find_Constraint()
{
	return	(this->*m_pfn_find_constraint)();
}

void ContactConstraint::_unite_do_something()
{
	srSystem * pLRoot = Constraint::UF_Find_System_PathCompression(pLSystem);
	srSystem * pRRoot = Constraint::UF_Find_System_PathCompression(pRSystem);

	if (pLRoot == pRRoot)
		return;

	if (pLRoot->m_UF_sz < pRRoot->m_UF_sz)
	{
		pLRoot->m_UF_id = pRRoot;
		pRRoot->m_UF_sz += pLRoot->m_UF_sz;
	}
	else
	{
		pRRoot->m_UF_id = pLRoot;
		pLRoot->m_UF_sz += pRRoot->m_UF_sz;
	}
}

void ContactConstraint::_unite_do_nothing()
{
}

inline void	ContactConstraint::UF_Unite()
{
	(this->*m_pfn_unite)();
}

void ContactConstraint::GetInformation(ConstraintInfo * info)
{
	// relative velocity
	GetRelVelocity(info->_rhs);

	int i;
	if (bFriction)
	{
		int tmp = 0;
		for (i = 0 ; i < nContactPts ; i++)
		{
			//-- lo, hi, findex
			info->_lo[tmp] = 0.0;
			info->_hi[tmp] = PINFINITY_BK;

			info->_lo[tmp+1] = -rFriction_coeff;
			info->_hi[tmp+1] = rFriction_coeff;

			info->_lo[tmp+2] = -rFriction_coeff;
			info->_hi[tmp+2] = rFriction_coeff;

			info->_findex[tmp+1] = tmp;
			info->_findex[tmp+2] = tmp;


			//-- rhs

			// error reduction (penetration)
			sr_real c = pContactPts[i]->Penetration - allowedpenetration;
			if (c < 0.0)
			{
				c = 0.0;
			}
			else
			{
				c *= (info->recip_timestep * erp_contact);
				if (c > maximum_erp_velocity)
				{
					c = maximum_erp_velocity;
				}
			}

			sr_real & neg_relvel = info->_rhs[tmp];

			// bounce
			if (bBounce)
			{
				if (neg_relvel > bouncing_threshold)
				{
					sr_real newc = rRestitution_coeff * neg_relvel;

					if (newc > c)
					{
						c = newc;
						if (c > maximum_bouncing_velocity)
						{
							c = maximum_bouncing_velocity;
						}
					}
				}
			}
			info->_rhs[tmp] += c;

			//// Tangent 방향의 constraint force는 warmstarting 안함
			////-- lambda
			//if (pContactPts[i]->lifetime)
			//	info->_lambda[tmp] = pContactPts[i]->lambda;
			//else
			//	info->_lambda[tmp] = 0.0;

			//info->_lambda[tmp+1] = 0.0;
			//info->_lambda[tmp+2] = 0.0;

			//-- lambda
			if (pContactPts[i]->lifetime)
			{
				info->_lambda[tmp] = pContactPts[i]->lambda[0];
				info->_lambda[tmp+1] = pContactPts[i]->lambda[1];
				info->_lambda[tmp+2] = pContactPts[i]->lambda[2];
			}
			else
			{
				info->_lambda[tmp] = 0.0;
				info->_lambda[tmp+1] = 0.0;
				info->_lambda[tmp+2] = 0.0;
			}

			tmp += 3;
		}
	}
	else //(!bFriction)
	{
		for (i = 0 ; i < nContactPts ; i++)
		{
			//-- lo, hi, findex
			info->_lo[i] = 0;
			info->_hi[i] = PINFINITY_BK;

			//-- rhs

			// error reduction (penetration)
			sr_real c = pContactPts[i]->Penetration - allowedpenetration;
			if (c < 0.0)
			{
				c = 0.0;
			}
			else
			{
				if (c > maximum_erp_velocity)
				{
					c = maximum_erp_velocity;
				}
				c *= (info->recip_timestep * erp_contact);
			}
			sr_real & neg_relvel = info->_rhs[i];

			// bounce
			if (bBounce)
			{
				if (neg_relvel > bouncing_threshold)
				{
					sr_real newc = rRestitution_coeff * neg_relvel;

					if (newc > c)
					{
						c = newc;

						if (c > maximum_bouncing_velocity)
						{
							c = maximum_bouncing_velocity;
						}
					}
				}
			}
			info->_rhs[i] += c;

			//// Tangent 방향의 constraint force는 warmstarting 안함
			//-- lambda
			//if (pContactPts[i]->lifetime)
			//	info->_lambda[i] = pContactPts[i]->lambda;
			//else
			//	info->_lambda[i] = 0.0;

			//-- lambda
			if (pContactPts[i]->lifetime)
				info->_lambda[i] = pContactPts[i]->lambda[0];
			else
				info->_lambda[i] = 0.0;

		}
	}
}

void ContactConstraint::_applyimpulse_FS_Right(int _idx)
{
	pRSystem->FS_Reset_Bias_T();
	pRSystem->FS_UpdateBiasImpulse(pRightMass, JacobianRight[_idx]);
	pRSystem->FS_UpdateDelVelocity();
}

void ContactConstraint::_applyimpulse_FS_Left(int _idx)
{
	pLSystem->FS_Reset_Bias_T();
	pLSystem->FS_UpdateBiasImpulse(pLeftMass,JacobianLeft[_idx]);
	pLSystem->FS_UpdateDelVelocity();
}

void ContactConstraint::_applyimpulse_FS_Both(int _idx)
{
	pLSystem->FS_Reset_Bias_T();
	pLSystem->FS_UpdateBiasImpulse(pLeftMass, JacobianLeft[_idx]);
	pLSystem->FS_UpdateBiasImpulse(pRightMass, JacobianRight[_idx]);
	pLSystem->FS_UpdateDelVelocity();
}

void ContactConstraint::_applyimpulse_L_Right(int _idx)
{
	pRSystem->L_UpdateDelVelocity(JacobianRight[_idx]);
}

void ContactConstraint::_applyimpulse_L_Left(int _idx)
{
	pLSystem->L_UpdateDelVelocity(JacobianLeft[_idx]);
}

void ContactConstraint::_applyimpulse_Both(int _idx)
{
	(this->*m_pfn_applyimpulse_Left)(_idx);
	(this->*m_pfn_applyimpulse_Right)(_idx);
}

void ContactConstraint::ApplyImpulse(int _idx)
{
	(this->*m_pfn_applyimpulse)(_idx);
}

inline void ContactConstraint::__getdelvelocity_Leftside(sr_real *sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = pLeftMass->m_DelVel * JacobianLeft[i];
	}
}

inline void ContactConstraint::__getdelvelocity_Rightside(sr_real *sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = pRightMass->m_DelVel * JacobianRight[i];
	}
}

inline void ContactConstraint::__getdelvelocity_Bothsides(sr_real *sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = pRightMass->m_DelVel * JacobianRight[i] + pLeftMass->m_DelVel * JacobianLeft[i];
	}
}

inline void ContactConstraint::__getdelvelocity_None(sr_real *sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = 0.0;
	}
}

void ContactConstraint::_getdelvelocity_BothMove(sr_real *sjari)
{
	if (pLSystem->m_bExcited)
	{
		if (pRSystem->m_bExcited)
			__getdelvelocity_Bothsides(sjari);
		else
			__getdelvelocity_Leftside(sjari);
	}
	else
	{
		if (pRSystem->m_bExcited)
			__getdelvelocity_Rightside(sjari);
		else
			__getdelvelocity_None(sjari);
	}
}

void ContactConstraint::_getdelvelocity_LeftMove(sr_real *sjari)
{
	if (pLSystem->m_bExcited)
		__getdelvelocity_Leftside(sjari);
	else
		__getdelvelocity_None(sjari);
}

void ContactConstraint::_getdelvelocity_RightMove(sr_real *sjari)
{
	if (pRSystem->m_bExcited)
		__getdelvelocity_Rightside(sjari);
	else
		__getdelvelocity_None(sjari);
}

void ContactConstraint::GetDelVelocity(sr_real * sjari)
{
	(this->*m_pfn_getdelvelocity)(sjari);
}

void ContactConstraint::_excite_BothSystem()
{
	pLSystem->ExciteSystem();
	pRSystem->ExciteSystem();
}

void ContactConstraint::_excite_LeftSystem()
{
	pLSystem->ExciteSystem();
}

void ContactConstraint::_excite_RightSystem()
{
	pRSystem->ExciteSystem();
}

void ContactConstraint::Excite()
{
	(this->*m_pfn_excite)();
}

void ContactConstraint::_unexcite_BothSystem()
{
	pLSystem->UnExciteSystem();
	pRSystem->UnExciteSystem();
}

void ContactConstraint::_unexcite_LeftSystem()
{
	pLSystem->UnExciteSystem();
}

void ContactConstraint::_unexcite_RightSystem()
{
	pRSystem->UnExciteSystem();
}

void ContactConstraint::UnExcite()
{
	(this->*m_pfn_unexcite)();
}

void ContactConstraint::_getrelvelocity_LeftMove(sr_real * sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = -(pLeftMass->m_Vel * JacobianLeft[i]);
	}
}

void ContactConstraint::_getrelvelocity_RightMove(sr_real * sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = -(pRightMass->m_Vel * JacobianRight[i]);
	}
}

void ContactConstraint::_getrelvelocity_BothMove(sr_real * sjari)
{
	for (int i = 0 ; i < nd ; i++)
	{
		sjari[i] = -(pRightMass->m_Vel * JacobianRight[i] + pLeftMass->m_Vel * JacobianLeft[i]);
	}
}

void ContactConstraint::GetRelVelocity(sr_real * sjari)
{
	(this->*m_pfn_getrelvelocity)(sjari);
}

void ContactConstraint::SetImpulse(sr_real * _lambda)
{
	int i;
	if (bFriction)
	{
		int tmp = 0;
		for (i = 0 ; i < nContactPts ; i++)
		{
			pContactPts[i]->lambda[0] = _lambda[tmp];
			pLeftMass->m_ConstraintImpulse += JacobianLeft[tmp] * _lambda[tmp];
			pRightMass->m_ConstraintImpulse += JacobianRight[tmp] * _lambda[tmp];

			tmp++;

			pContactPts[i]->lambda[1] = _lambda[tmp];
			pLeftMass->m_ConstraintImpulse += JacobianLeft[tmp] * _lambda[tmp];
			pRightMass->m_ConstraintImpulse += JacobianRight[tmp] * _lambda[tmp];

			tmp++;

			pContactPts[i]->lambda[2] = _lambda[tmp];
			pLeftMass->m_ConstraintImpulse += JacobianLeft[tmp] * _lambda[tmp];
			pRightMass->m_ConstraintImpulse += JacobianRight[tmp] * _lambda[tmp];

			tmp++;
		}
	}
	else //(!bFriction)
	{
		for (i = 0 ; i < nContactPts ; i++)
		{
			pContactPts[i]->lambda[0] = _lambda[i];
			pLeftMass->m_ConstraintImpulse += JacobianLeft[i] * _lambda[i];
			pRightMass->m_ConstraintImpulse += JacobianRight[i] * _lambda[i];
		}
	}
}
//**********************************************************************//


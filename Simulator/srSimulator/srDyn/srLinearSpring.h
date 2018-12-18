/*
 *  srLinearSpring.h
 *
 *  Created by Jaeyoung Haan on 3/12/09.
 *  Copyright 2009 Robotics Lab @ SNU. All rights reserved.
 *
 */

#ifndef SRLIB_LINEAR_SPRING
#define SRLIB_LINEAR_SPRING

#include "srDyn/srSpring.h"

/*!
 \class srLinearSpring
 \brief class to represent a linear spring-damper element.

		srLinearSpring represents a spring-damper moving linearly.
		srLinearSpring does not take account of any obstacles between both ends.
 */
class srLinearSpring : public srSpring
{
protected:
	/*!
	 Spring coefficient. This is usually denoted as $K$.
	 Unit is $N/m$.
	 */
	sr_real	m_K;
	/*!
	 Damping coefficient. This is usually denoted as $C$.
	 Unit is $Ns/m$. Default is zero.
	 */
	sr_real	m_C;
	/*!
	 Length of spring when it is not elongated or compressed.
	 If you do not specify this, it will be set to be a length
	 of when it connected.
	 */
	sr_real	m_InitialLength;
	
	/*!
	 Length of spring in current state.
	 */
	sr_real	m_CurrentLength;

	/*!
	 Resultant spring-damper force in global coordinates.
	 */
	Vec3	m_Force;

	/*!
	 Directional Vector of force. This is a normal vector of m_Force.
	 */
	Vec3	m_DirectionalVector;

	/*!
	 Both belows are position vector of the attach point of each end of spring-damper on each link
	 w.r.t. each local coordinates of link.
	 Though there `m_Left(Right)LinkToSpring' member variables in super class `srSpring',
	 position vectors are additionally declared because these are frequently refered, 
	 */
	Vec3	m_LeftPosition;
	Vec3	m_RightPosition;

public:
	/*!
		Default constructor.
	 */
		srLinearSpring();
	/*!
	 Set spring coefficient value.
	 */
	void SetK(sr_real );
	/*!
	 Set damping coefficient value.
	 */
	void SetC(sr_real );
	/*!
	 Set spring initial length.
	 */
	void SetInitialLength(sr_real il = -1.0);
	/*! 
	 Get current spring length.
	 */
	sr_real& GetLength(void);
	
	/*!
	 Initialize its length.
	 */
	void Initialize(void);
	/*!
	 Set the attach position on the left link w.r.t. the local coordinates of left link.
	 In case of `SE(3)' argument, orientation will not be taken account.
	 */
	void SetLeftLinkPosition(Vec3 );
	void SetLeftLinkPosition(SE3 );
	/*!
	 Set the attach position on the right link w.r.t. the local coordinates of right link.
	 In case of `SE(3)' argument, orientation will not be taken account.
	 */
	void SetRightLinkPosition(Vec3 );
	void SetRightLinkPosition(SE3 );
	/*!
	 Update a spring length and return it.
	 */
	sr_real& UpdateLength(void);
	/*!
	 Update a relative speed of the right end point to the left.
	 Velocity of the right end point to the left is projected on to the line along the spring.
	 Positive value means the spring is in elogation, negative means it is under compression.
	 */
	sr_real UpdateSpeed(void);
	/*!
	 Update spring/damper force. The generalized force exerts differently on each link. 
	 */
	void UpdateForce(void);
	dse3 GetForceOnLeft(void);
	dse3 GetForceOnRight(void);
};

#endif

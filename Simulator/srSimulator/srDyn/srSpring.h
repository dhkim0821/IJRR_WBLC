/*
 *  srSpring.h
 *
 *  Created by Jaeyoung Haan on 3/12/09.
 *  Copyright 2009 Robotics Lab @ SNU. All rights reserved.
 *
 */

#ifndef SRLIB_SPRING
#define SRLIB_SPRING

#include "srLink.h"

/*!
 \class srSpring
 \brief Abstract class to represent various spring-damper's.

		Spring-damper is a bi-connective element.
		Spring-damper is belonged to space not to any system, link, or joint,
		since it can connect any pair of links.
		Dynamics engine calls `srSpring::Initialize()' function in preprocess step,
		and calls `srSpring::UpdateForce()', `srSpring::GetForceOnLeft()' and `srSpring::GetForceOnRight()' functions
		every simulation step in order to exert spring-damper force on corresponding links.
 */
class srSpring : public srEntity
{
protected:
	/*!
		Link pointers connected to each end of spring.
		In case of spring, left-right is more suitable than parent-child in connectivity with links.
//		But for consistency with other class variables and methods such as a joint class,
//		`parent-child' is used for names of variables.		
	 */
	srLink*	m_LeftLink;
	srLink* m_RightLink;
	/*!
		Position of an attached point to spring w.r.t. the center of link frame.
		Do not need orientation up to now.
	 */
	SE3		m_LeftLinkToSpring;
	SE3		m_RightLinkToSpring;

	sr_real	m_Active;

public:
	/*!
		Default constructor.
	 */
			srSpring();
	/*!
		Connect link to spring. Spring has left and right links rather than parent and child of joint.
		But for consistency of method names `SetParentLink' and `SetChildLink' are also declared.
		They do exactly same thing as `SetLeftLink' and `SetRightLink'.
		A `Left Link' is supposed to be a `Parent Link'.
	 */
	void	SetLeftLink (srLink* );
	void	SetRightLink(srLink* );
	/*!
		Get link pointer.
	 */
	srLink*	GetLeftLink();
	srLink*	GetRightLink();
	/*!
		Set attach position of spring on a link w.r.t. the center of a link.
		For these functions as same as `SetLeftLink' method, there are additional functions
		doing same functionality.
	 */
	virtual void SetLeftLinkPosition (SE3 );
	virtual void SetRightLinkPosition(SE3 );
	virtual void SetLeftLinkPosition (Vec3 );
	virtual void SetRightLinkPosition(Vec3 );

	/*!
		Get the position of the left end of the spring in global coordinates.
	 */
	virtual Vec3 GetLeftEnd();
	/*!
		Get the position of the right end of the spring in global coordinates.
	*/
	virtual Vec3 GetRightEnd();
	/*!
		Activate(connect)/Deactivate(cut) spring-damper.
		TRUE for activate, FALSE for deactivate.
	 */
	virtual void Activate(bool );
	/*!
		Return the activation state.
	 */
	bool	IsActive();
	/*!
		Initialize its length.
	 */
	virtual void Initialize() = 0;
	/*!
		Update spring/damper force. The generalized force exerts differently on each link. 
	 */
	virtual void UpdateForce() = 0;
	virtual dse3 GetForceOnLeft () = 0;
	virtual dse3 GetForceOnRight() = 0;
};

#endif

/*
 *  srLinearSpring.cpp
 *  Xcode3.0
 *
 *  Created by Jaeyoung Haan on 3/12/09.
 *  Copyright 2009 ?œìš¸?€?™êµ ê³µê³¼?€??ë¡œë´‡?ë™?”ì—°êµ¬ì‹¤. All rights reserved.
 *
 */

#include "srLinearSpring.h"

srLinearSpring::srLinearSpring()
: m_K(1.0)
, m_C(0.0)
, m_InitialLength(-1.0)
, m_CurrentLength(-1.0)
, m_Force(Vec3(0.0))
, m_DirectionalVector(Vec3(0.0))
, m_LeftPosition(Vec3(0.0))
, m_RightPosition(Vec3(0.0))
{	
};

void srLinearSpring::SetK(sr_real k)
{
	if(k >= 0.0)
		m_K = k;
};

void srLinearSpring::SetC(sr_real c)
{
	if(c >= 0.0)
		m_C = c;
};

void srLinearSpring::SetInitialLength(sr_real l)
{
	if(l >= 0.0)
		m_InitialLength = l;
};

sr_real& srLinearSpring::GetLength()
{
	return m_CurrentLength;
};

void srLinearSpring::Initialize()
{
	if(m_InitialLength < 0.0) {
		SetInitialLength(UpdateLength());
	}
};

void srLinearSpring::SetLeftLinkPosition(Vec3 lp)
{
	m_LeftPosition = lp;
	srSpring::SetLeftLinkPosition(m_LeftPosition);
};

void srLinearSpring::SetRightLinkPosition(Vec3 rp)
{
	m_RightPosition = rp;
	srSpring::SetRightLinkPosition(m_RightPosition);
};

void srLinearSpring::SetLeftLinkPosition(SE3 lp)
{
	m_LeftPosition = lp.GetPosition();
	srSpring::SetLeftLinkPosition(m_LeftPosition);
};

void srLinearSpring::SetRightLinkPosition(SE3 rp)
{
	m_RightPosition = rp.GetPosition();
	srSpring::SetRightLinkPosition(m_RightPosition);
};

//* Distance from the left end to the right end.
sr_real& srLinearSpring::UpdateLength()
{
	m_DirectionalVector = m_RightLink->m_Frame * m_RightPosition - m_LeftLink->m_Frame * m_LeftPosition;
	m_CurrentLength = m_DirectionalVector.Normalize();
	return m_CurrentLength;
};

sr_real srLinearSpring::UpdateSpeed()
{
	Vec3 dv = m_RightLink->GetLinearVel(m_RightPosition) - m_LeftLink->GetLinearVel(m_LeftPosition);
	sr_real ds = Inner(m_DirectionalVector, dv);
	return ds;
};

void srLinearSpring::UpdateForce()
{
	m_Force = (m_K * (UpdateLength() - m_InitialLength) + m_C * UpdateSpeed()) * m_DirectionalVector;
	m_Force *= m_Active;
};

dse3 srLinearSpring::GetForceOnLeft()
{
	return InvdAd(m_LeftPosition, InvRotate(m_LeftLink->m_Frame, m_Force));
};

dse3 srLinearSpring::GetForceOnRight()
{
	return InvdAd(m_RightPosition, InvRotate(m_RightLink->m_Frame, -m_Force));
};

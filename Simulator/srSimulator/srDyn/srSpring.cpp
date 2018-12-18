/*
 *  srSpring.cpp
 *  Xcode3.0
 *
 *  Created by Jaeyoung Haan on 3/12/09.
 *  Copyright 2009 ?œìš¸?€?™êµ ê³µê³¼?€??ë¡œë´‡?ë™?”ì—°êµ¬ì‹¤. All rights reserved.
 *
 */

#include "srSpring.h"

srSpring::srSpring()
: m_LeftLink(NULL)
, m_RightLink(NULL)
, m_LeftLinkToSpring(SE3())
, m_RightLinkToSpring(SE3())
, m_Active(1.0)
{	
};

void srSpring::SetLeftLink(srLink* lLink)
{
	m_LeftLink = lLink;
};

void srSpring::SetRightLink(srLink* rLink)
{
	m_RightLink = rLink;
};

void srSpring::SetLeftLinkPosition(SE3 lf)
{
	m_LeftLinkToSpring = lf;
};

void srSpring::SetRightLinkPosition(SE3 rf)
{
	m_RightLinkToSpring = rf;
};

void srSpring::SetLeftLinkPosition(Vec3 lf)
{
	m_LeftLinkToSpring = SE3(lf);
};

void srSpring::SetRightLinkPosition(Vec3 rf)
{
	m_RightLinkToSpring = SE3(rf);
};

srLink* srSpring::GetLeftLink()
{
	return m_LeftLink;
};

srLink* srSpring::GetRightLink()
{
	return m_RightLink;
};

void srSpring::Activate(bool b)
{
	m_Active = (b) ? 1.0 : 0.0;
}

bool srSpring::IsActive()
{
	return (m_Active == 1.0);
}
Vec3 srSpring::GetLeftEnd()
{
	return GetLeftLink()->GetFrame() * m_LeftLinkToSpring.GetPosition();
};

Vec3 srSpring::GetRightEnd()
{
	return GetRightLink()->GetFrame() * m_RightLinkToSpring.GetPosition();
};
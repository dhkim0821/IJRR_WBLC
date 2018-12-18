#include <memory.h>
#include "srDyn/srCollision.h"
#include "srDyn/srLink.h"

#include "LieGroup/LieGroup.h"

//**********************************************************************//
// KIN
srCollision::srCollision()
{
	m_LocalFrame = SE3();
	m_pLink = NULL;

	m_BoundingRadius = 0.0;
}
srCollision::~srCollision()
{
}

SE3& srCollision::GetLocalFrame()
{
	return m_LocalFrame;
}

void srCollision::SetLocalFrame(SE3 v)
{
	m_LocalFrame = v;
}

void srCollision::SetLocalFrame(SE3& v)
{
	m_LocalFrame = v;
}

srGeometryInfo::SHAPETYPE srCollision::GetCollisionShape()
{
	return m_GeomInfo.m_Type;
}

Vec3& srCollision::GetDimension()
{
	return m_GeomInfo.m_Dimension;
}

void srCollision::UpdateFrame()
{
	m_Frame = m_pLink->m_Frame * m_LocalFrame;
}

sr_real& srCollision::GetBoundingRadius()
{
	return m_BoundingRadius;
}

void srCollision::UpdateBoundingRadius()
{
	switch(GetCollisionShape())
	{
	case srGeometryInfo::BOX:
		m_BoundingRadius = 0.5 * sqrt( GetDimension()[0] * GetDimension()[0]
		+ GetDimension()[1] * GetDimension()[1]
		+ GetDimension()[2] * GetDimension()[2] );

		break;
	case srGeometryInfo::SPHERE:
		m_BoundingRadius = 0.5 * GetDimension()[0];

		break;
	case srGeometryInfo::CYLINDER:
		m_BoundingRadius = 0.5 * sqrt( GetDimension()[0] * GetDimension()[0]
		+ GetDimension()[1] * GetDimension()[1] );

		break;
	case srGeometryInfo::CAPSULE:
		m_BoundingRadius = 0.5 * (GetDimension()[0] +  GetDimension()[1]);

		break;
	case srGeometryInfo::PLANE:
		m_BoundingRadius = 10000; // TEMP

		break;
	default:
		break;
	}
}

//-- Ray Detection
bool srCollision::RayDetection(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	return	(this->*m_pfn_RayDetect)(point,direction,range,dist);
}

void srCollision::UpdateRayDetectionFtn()
{
	switch(GetCollisionShape()) {
		case srGeometryInfo::BOX:
			m_pfn_RayDetect = &srCollision::_RayToBox;
			break;

		case srGeometryInfo::SPHERE:
			m_pfn_RayDetect = &srCollision::_RayToSphere;
			break;

		case srGeometryInfo::CYLINDER:
			m_pfn_RayDetect = &srCollision::_RayToCylinder;
			break;

		case srGeometryInfo::CAPSULE:
			m_pfn_RayDetect = &srCollision::_RayToCapsule;
			break;

		case srGeometryInfo::PLANE:
			m_pfn_RayDetect = &srCollision::_RayToPlane;
			break;

		default:
			m_pfn_RayDetect = &srCollision::_RayToUser;
			break;
	}
}

//-- Touch Detection
bool srCollision::TouchDetection(Vec3& position, sr_real radius, Vec3& point, Vec3& normal, sr_real& penetration)
{
	return	(this->*m_pfn_TouchDetect)(position,radius,point,normal,penetration);
}

void srCollision::UpdateTouchDetectionFtn()
{
	switch(GetCollisionShape()) {
		case srGeometryInfo::BOX:
			m_pfn_TouchDetect = &srCollision::_TouchToBox;
			break;

		case srGeometryInfo::SPHERE:
			m_pfn_TouchDetect = &srCollision::_TouchToSphere;
			break;

		case srGeometryInfo::CYLINDER:
			m_pfn_TouchDetect = &srCollision::_TouchToCylinder;
			break;

		case srGeometryInfo::CAPSULE:
			m_pfn_TouchDetect = &srCollision::_TouchToCapsule;
			break;

		case srGeometryInfo::PLANE:
			m_pfn_TouchDetect = &srCollision::_TouchToPlane;
			break;

		default:
			m_pfn_TouchDetect = &srCollision::_TouchToUser;
			break;
	}
}

//-- Sonar Detection
bool srCollision::SonarDetection(SE3& T0, sr_real & range , sr_real & angle, bool & res)
{
	return	(this->*m_pfn_SonarDetect)(T0,range,angle,res);
}

void srCollision::UpdateSonarDetectionFtn()
{
	switch(GetCollisionShape()) {
		case srGeometryInfo::BOX:
			m_pfn_SonarDetect = &srCollision::_SonarToBox;
			break;

		case srGeometryInfo::SPHERE:
			m_pfn_SonarDetect = &srCollision::_SonarToSphere;
			break;

		case srGeometryInfo::CYLINDER:
			m_pfn_SonarDetect = &srCollision::_SonarToCylinder;
			break;

		case srGeometryInfo::CAPSULE:
			m_pfn_SonarDetect = &srCollision::_SonarToCapsule;
			break;

		case srGeometryInfo::PLANE:
			m_pfn_SonarDetect = &srCollision::_SonarToPlane;
			break;

		default:
			m_pfn_SonarDetect = &srCollision::_SonarToUser;
			break;
	}
}
//**********************************************************************//

bool RayToSphere(Vec3& point,Vec3& direction, sr_real& range, 
				 const SE3 &T, sr_real r,
				 sr_real& dist)
{
	Vec3 q(&T[9]);
	Vec3 p1 = range * direction;

	sr_real a,b,c;
	a = Inner(p1,p1);
	b = 2*Inner(p1,point-q);
	c = Inner(point-q,point-q) - r*r;
	sr_real maxt, mint;
	sr_real rt = b*b - 4*a*c;
	if(rt < 0){
		dist = -1;
		return false;
	}
	else{
		maxt = (-b + sqrt(rt))/(2*a);
		mint = (-b - sqrt(rt))/(2*a);
		if(maxt >= range)
			maxt = range;
		if(mint <= 0)
			mint = 0;
		if(mint>maxt){
			dist = -1;
			return false;
		}
		else {
			dist = mint*range;
			return true;
		}
	}
}

bool RayToCylinder(Vec3& point, Vec3& direction, sr_real& range, 
				   const SE3 &T, sr_real r, sr_real h,
				   sr_real& dist)
{
	Vec3 CylinderCenter(&T[9]);
	Vec3 CylinderAxis(&T[6]);
	Vec3 CylinderTop = CylinderCenter + 0.5 * h * CylinderAxis;
	Vec3 CylinderBottom = CylinderCenter - 0.5 * h * CylinderAxis;


	sr_real mint,maxt;
	sr_real a,b,c,d;
	sr_real mint1=0,maxt1=-1;
	Vec3 p1 = range * direction;

	a = Inner(point-CylinderBottom,CylinderAxis);
	b = Inner(p1,CylinderAxis);
	c = SquareSum(CylinderAxis);
	if(b>0){
		mint1 = a/c;
		maxt1 = (a+b)/c;
	}else{
		mint1 = (a+b)/c;
		maxt1 = a/c;
	}
	if(mint1 < 0) mint1 = 0;
	if(maxt1 > 1) maxt1 = 1;
	if(mint1 > maxt1) {
		dist = range;
		return false;
	}
	if(b==0){
		mint = 0; maxt = 1;
	}else if(b>0){
		mint = (mint1*c-a)/b;
		maxt = (maxt1*c-a)/b;
	}else{
		mint = (maxt1*c-a)/b;
		maxt = (mint1*c-a)/b;
	}
	if(mint < 0) mint = 0;
	if(maxt > 1) maxt = 1;
	if(mint > maxt) {
		dist = range;
		return false;
	}
	Vec3 v1,v2;
	v1 = point - CylinderBottom - (CylinderAxis*(a/c));
	v2 = p1-((b/c)*CylinderAxis);

	if(Norm(v2) == 0)
		if(Norm(v1)<r){
			dist = mint*range;
			return true;
		}
		else{
			dist = range;
			return false;
		}
		a = SquareSum(v2);
		b = 2 * Inner(v1,v2);
		c = SquareSum(v1) - r*r;
		d = b*b-4*a*c;
		if(a==0 || d<0){
			dist = range;
			return false;
		}
		else{
			mint1 = (-b - sqrt(d))/(2*a);
			maxt1 = (-b + sqrt(d))/(2*a);
		}
		if(mint1 > mint) mint = mint1;
		if(maxt1 < maxt) maxt = maxt1;
		if(mint > maxt){ 
			dist = range;
			return false;
		}
		dist = mint*range;
		return true;
}

//////////////////////////////////////////////////////////////////////////
// RayDetection

void Inv3by3(sr_real* x, sr_real* b)	// x * b = I
{
	sr_real a11 = x[4]*x[8]-x[5]*x[7];
	sr_real a12 = x[1]*x[8]-x[2]*x[7];
	sr_real a13 = x[1]*x[5]-x[2]*x[4];
	sr_real det = x[0]*a11-x[3]*a12+x[6]*a13;
	det = 1.0/det;

	b[0]= a11*det;	b[3]=-(x[3]*x[8]-x[5]*x[6])*det;	b[6]= (x[3]*x[7]-x[4]*x[6])*det;
	b[1]=-a12*det;	b[4]= (x[0]*x[8]-x[2]*x[6])*det;	b[7]=-(x[0]*x[7]-x[1]*x[6])*det;
	b[2]= a13*det;	b[5]=-(x[0]*x[5]-x[2]*x[3])*det;	b[8]= (x[0]*x[4]-x[1]*x[3])*det;
}

void Mult3by3(sr_real* a, sr_real* b, sr_real* r)	// a*b=r, a,b,r: 3 by 3 matrix
{
	r[0]=a[0]*b[0]+a[3]*b[1]+a[6]*b[2];	r[0]=a[0]*b[3]+a[3]*b[4]+a[6]*b[5];	r[0]=a[0]*b[6]+a[3]*b[7]+a[6]*b[8];
	r[1]=a[1]*b[0]+a[4]*b[1]+a[7]*b[2];	r[1]=a[1]*b[3]+a[4]*b[4]+a[7]*b[5];	r[1]=a[1]*b[6]+a[4]*b[7]+a[7]*b[8];
	r[2]=a[2]*b[0]+a[5]*b[1]+a[8]*b[2];	r[2]=a[2]*b[3]+a[5]*b[4]+a[8]*b[5];	r[2]=a[2]*b[6]+a[5]*b[7]+a[8]*b[8];
}

void MultMV3by3(sr_real*a, sr_real*v, sr_real*r)		// a*v=r, a: 3 by 3 matrix, v: 3-dim vector, r:3-dim vector
{
	r[0]=a[0]*v[0]+a[3]*v[1]+a[6]*v[2];
	r[1]=a[1]*v[0]+a[4]*v[1]+a[7]*v[2];
	r[2]=a[2]*v[0]+a[5]*v[1]+a[8]*v[2];
}

bool srCollision::_RayToBox(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	SE3 & T = GetFrame();
	Vec3 & size = GetDimension();

	Vec3 p1 = range * direction;

	static sr_real A[9];						//static RMatrix A(3,3);
	static sr_real InvA[9];					//inverse matrix of A
	Vec3 q1(&T[0]),q2(&T[3]),q3(&T[6]);
	q1 *= size[0];
	q2 *= size[1];
	q3 *= size[2];

	memcpy(&A[0],&q1[0],sizeof(sr_real)*3);	//memcpy(&A(0,0),&q1[0],sizeof(sr_real)*3);
	memcpy(&A[3],&q2[0],sizeof(sr_real)*3);	//memcpy(&A(0,1),&q2[0],sizeof(sr_real)*3);
	memcpy(&A[6],&q3[0],sizeof(sr_real)*3);	//memcpy(&A(0,2),&q3[0],sizeof(sr_real)*3);

	static sr_real B[3];						//static RMatrix B(3,1);
	memcpy(&B[0],&p1[0],sizeof(sr_real)*3);	//memcpy(&B(0,0),&p1[0],sizeof(sr_real)*3);

	Vec3 Center(&T[9]);
	Vec3 q= Center-0.5*(q1+q2+q3);
	static Vec3 v;
	v = q-point;

	static sr_real PC[3];						//static RMatrix C(3,1);
	static sr_real C[3];						// InvA*PC=C;
	memcpy(&PC[0],&v[0],sizeof(sr_real)*3);	//memcpy(&C(0,0),&v[0],sizeof(sr_real)*3);

	Inv3by3(A, InvA);						//A = Inv(A);

	static sr_real Z[3];						//static RMatrix Z(3,1);
											// InvA*B=Z

	MultMV3by3(InvA, B, Z);					//Z = A * B;
	MultMV3by3(InvA, PC, C);				//C = A * C;

	static sr_real minVal, maxVal;
	sr_real minGlobal = -10000, maxGlobal = 10000;
	for(int i=0;i<3;i++){
		if(Z[i] > 0){								//if(Z(i,0) > 0){
			minVal = C[i] / Z[i];					//	minVal = C(i,0) / Z(i,0);
			maxVal = (C[i]+1) / Z[i];				//	maxVal = (C(i,0)+1) / Z(i,0);
		}else if(Z[i] == 0){						//}else if(Z(i,0) == 0){
			if(C[i] <= 0 && C[i] >= -1){			//	if(C(i,0) <= 0 && C(i,0) >= -1){
				minVal = minGlobal;					//		minVal = minGlobal;
				maxVal = maxGlobal;					//		maxVal = maxGlobal;
			}else{									//	}else{
				dist = range;						//		dist = range;
				return false;						//		return false;
			}										//	}
		}else{										//}else{
			minVal = (C[i]+1) / Z[i];				//	minVal = (C(i,0)+1) / Z(i,0);
			maxVal = C[i] / Z[i];					//	maxVal = C(i,0) / Z(i,0);
		}											//}
		if(minGlobal < minVal) minGlobal = minVal;
		if(maxGlobal > maxVal) maxGlobal = maxVal;
	}
	if(minGlobal < 0) minGlobal = 0;
	if(maxGlobal > 1) maxGlobal = 1;
	if(maxGlobal >= minGlobal){
		dist = minGlobal*range;
		return true;
	}
	else{
		dist = range;
		return false;
	}
}

bool srCollision::_RayToSphere(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	SE3 & T = GetFrame();
	sr_real r = 0.5*GetDimension()[0];
	//

	Vec3 q(&T[9]);
	Vec3 p1 = range * direction;

	sr_real a,b,c;
	a = Inner(p1,p1);
	b = 2*Inner(p1,point-q);
	c = Inner(point-q,point-q) - r*r;
	sr_real maxt, mint;
	sr_real rt = b*b - 4*a*c;
	if(rt < 0){
		dist = -1;
		return false;
	}
	else{
		maxt = (-b + sqrt(rt))/(2*a);
		mint = (-b - sqrt(rt))/(2*a);
		if(maxt >= range)
			maxt = range;
		if(mint <= 0)
			mint = 0;
		if(mint>maxt){
			dist = -1;
			return false;
		}
		else {
			dist = mint*range;
			return true;
		}
	}
}

bool srCollision::_RayToCylinder(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	SE3 & T = GetFrame();
	sr_real r = 0.5*GetDimension()[0];
	sr_real h = GetDimension()[1];
	//


	Vec3 CylinderCenter(&T[9]);
	Vec3 CylinderAxis(&T[6]);
	Vec3 CylinderTop = CylinderCenter + 0.5 * h * CylinderAxis;
	Vec3 CylinderBottom = CylinderCenter - 0.5 * h * CylinderAxis;


	sr_real mint,maxt;
	sr_real a,b,c,d;
	sr_real mint1=0,maxt1=-1;
	Vec3 p1 = range * direction;

	a = Inner(point-CylinderBottom,CylinderAxis);
	b = Inner(p1,CylinderAxis);
	c = SquareSum(CylinderAxis);
	if(b>0){
		mint1 = a/c;
		maxt1 = (a+b)/c;
	}else{
		mint1 = (a+b)/c;
		maxt1 = a/c;
	}
	if(mint1 < 0) mint1 = 0;
	if(maxt1 > 1) maxt1 = 1;
	if(mint1 > maxt1) {
		dist = range;
		return false;
	}
	if(b==0){
		mint = 0; maxt = 1;
	}else if(b>0){
		mint = (mint1*c-a)/b;
		maxt = (maxt1*c-a)/b;
	}else{
		mint = (maxt1*c-a)/b;
		maxt = (mint1*c-a)/b;
	}
	if(mint < 0) mint = 0;
	if(maxt > 1) maxt = 1;
	if(mint > maxt) {
		dist = range;
		return false;
	}
	Vec3 v1,v2;
	v1 = point - CylinderBottom - (CylinderAxis*(a/c));
	v2 = p1-((b/c)*CylinderAxis);

	if(Norm(v2) == 0)
		if(Norm(v1)<r){
			dist = mint*range;
			return true;
		}
		else{
			dist = range;
			return false;
		}
		a = SquareSum(v2);
		b = 2 * Inner(v1,v2);
		c = SquareSum(v1) - r*r;
		d = b*b-4*a*c;
		if(a==0 || d<0){
			dist = range;
			return false;
		}
		else{
			mint1 = (-b - sqrt(d))/(2*a);
			maxt1 = (-b + sqrt(d))/(2*a);
		}
		if(mint1 > mint) mint = mint1;
		if(maxt1 < maxt) maxt = maxt1;
		if(mint > maxt){ 
			dist = range;
			return false;
		}
		dist = mint*range;
		return true;
}

bool srCollision::_RayToCapsule(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	SE3 & T = GetFrame();
	sr_real r = 0.5*GetDimension()[0];
	sr_real h = GetDimension()[1];
	//


	Vec3 CapsuleCenter(&T[9]);
	Vec3 CapsuleAxis(&T[6]);
	Vec3 CapsuleTop = CapsuleCenter + 0.5 * h * CapsuleAxis;
	Vec3 CapsuleBottom = CapsuleCenter - 0.5 * h * CapsuleAxis;

	sr_real dist1,dist2,dist3;
	SE3 T1(CapsuleTop);
	SE3 T2(CapsuleBottom);
	bool rt1 = RayToSphere(point,direction,range,		T1,r,dist1);
	bool rt2 = RayToSphere(point,direction,range,		T2,r,dist2);
	bool rt3 = RayToCylinder(point,direction,range,		T,r,h,dist3);

	if(rt1) dist = dist1;
	if(rt2 && dist2 < dist) dist = dist2;
	if(rt3 && dist3 < dist) dist = dist3;

	if(rt1 || rt2 || rt3)
	{
		return true;
	}
	return false;
}

bool srCollision::_RayToPlane(Vec3& point, Vec3& direction, sr_real& range, sr_real& dist)
{
	SE3 & T = GetFrame();
	//


	Vec3 center(&T[9]);
	Vec3 axis(&T[6]);
	Vec3 p1 = range * direction;
	sr_real d1, d2;
	d1=Inner(point-center,axis);
	d2=Inner(point+p1-center,axis);

	if (d1*d2<0) {
		d2 *= -1;
		dist = (d1/(d1+d2))*range;
		return true;
	}
	dist = range;
	return false;
}

bool srCollision::_RayToUser(Vec3& /*point*/, Vec3& /*direction*/, sr_real& /*range*/, sr_real& /*dist*/)
{
	return false;
}

//////////////////////////////////////////////////////////////////////////
// TouchDetection
// normal : Geom to Touch     <-
bool srCollision::_TouchToBox(Vec3& c0, sr_real r0, Vec3& point, Vec3& normal, sr_real& penetration)
{
	// Sphere

	// Box
	SE3 & T1 = GetFrame();
	Vec3 size = 0.5*GetDimension();

	bool inside_box = true;

	// clipping a center of the sphere to a boundary of the box
	Vec3 p = T1 % c0;

	if (p[0] < -size[0]) { p[0] = -size[0]; inside_box = false; }
	if (p[0] >  size[0]) { p[0] =  size[0]; inside_box = false; }

	if (p[1] < -size[1]) { p[1] = -size[1]; inside_box = false; }
	if (p[1] >  size[1]) { p[1] =  size[1]; inside_box = false; }

	if (p[2] < -size[2]) { p[2] = -size[2]; inside_box = false; }
	if (p[2] >  size[2]) { p[2] =  size[2]; inside_box = false; }




	if ( inside_box )
	{
		// find nearest side from the sphere center
		sr_real min = size[0] - fabs(p[0]);
		sr_real tmin = size[1] - fabs(p[1]);
		int idx = 0;

		if ( tmin < min )
		{
			min = tmin;
			idx = 1;
		}
		tmin = size[2] - fabs(p[2]);
		if ( tmin < min )
		{
			min = tmin;
			idx = 2;
		}

		normal = 0.0;
		normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
		normal = Rotate(T1, normal);
		penetration = min + r0;

		point = c0;
		return true;
	}


	point = T1 * p;
	normal = c0 - point;
	sr_real mag = sqrt(SquareSum(normal));
	penetration = r0 - mag;

	if (penetration < 0.0)
	{
		return false;
	}

	if (mag > SR_EPS)
	{
		normal /= mag;
		return true;
	}
	else
	{
		sr_real min = size[0] - fabs(p[0]);
		sr_real tmin = size[1] - fabs(p[1]);
		int idx = 0;

		if ( tmin < min )
		{
			min = tmin;
			idx = 1;
		}
		tmin = size[2] - fabs(p[2]);
		if ( tmin < min )
		{
			min = tmin;
			idx = 2;
		}
		normal = 0.0;
		normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
		normal = Rotate(T1, normal);

		return true;
	}
}

bool srCollision::_TouchToSphere(Vec3& c0, sr_real r0, Vec3& point, Vec3& normal, sr_real& penetration)
{
	// Sphere1

	// Sphere2
	SE3 & c1 = GetFrame();
	sr_real r1 = 0.5*GetDimension()[0];

	sr_real rsum = r0 + r1;
	normal.SetValues(c0[0] - c1[9], c0[1] - c1[10], c0[2] - c1[11]);
	sr_real normal_sqr = SquareSum(normal);

	if ( normal_sqr > rsum * rsum )
	{
		return false;
	}

	r0 /= rsum;
	r1 /= rsum;

	point.SetValues(r1 * c0[0] + r0 * c1[9], r1 * c0[1] + r0 * c1[10], r1 * c0[2] + r0 * c1[11]);

	if (normal_sqr <SR_EPS)
	{
		normal.SetValues(0.0,0.0,1.0);
		penetration = rsum;
		return true;
	}

	normal_sqr = sqrt(normal_sqr);
	normal /= normal_sqr;
	penetration = rsum - normal_sqr;
	return true;
}

bool srCollision::_TouchToCylinder(Vec3& /*position*/, sr_real /*radius*/, Vec3& /*point*/, Vec3& /*normal*/, sr_real& /*penetration*/)
{
	// 미구현
	return false;
}

bool srCollision::_TouchToCapsule(Vec3& c1, sr_real r1, Vec3& point, Vec3& normal, sr_real& penetration)
{
	// CapsuleSphere normal 뒤집힘

	// Sphere

	// Capsule
	SE3 & T0 = GetFrame();
	sr_real r0 = 0.5*GetDimension()[0];
	sr_real h0 = GetDimension()[1];


	Vec3 dir(&T0[6]);
	Vec3 c0(&T0[9]);

	sr_real t = Inner(dir, c1 - c0);

	h0 *= 0.5;
	if ( t > h0 ) t = h0;
	if ( t < -h0 ) t = -h0;

	dir *= t;
	c0 += dir;


	sr_real rsum = r1 + r0;
	normal.SetValues(c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]);
	sr_real normal_sqr = SquareSum(normal);

	if ( normal_sqr > rsum * rsum)
	{
		return false;
	}

	r1 /= rsum;
	r0 /= rsum;


	point.SetValues(r1 * c0[0] + r0 * c1[0], r1 * c0[1] + r0 * c1[1], r1 * c0[2] + r0 * c1[2]);

	if (normal_sqr < SR_EPS)
	{
		normal.SetValues(0.0,0.0,1.0);
		penetration = rsum;
		return true;
	}

	normal_sqr = sqrt(normal_sqr);
	normal /= normal_sqr;
	penetration = rsum - normal_sqr;

	return true;
}

bool srCollision::_TouchToPlane(Vec3& c1, sr_real r1, Vec3& point, Vec3& normal, sr_real& penetration)
{
	// Plane
	SE3 & T0 = GetFrame();

	// Sphere

	Vec3 c0(&T0[9]);
	Vec3 pt = c0 - c1;
	normal.SetValues(T0[6],T0[7],T0[8]);
	penetration = Inner(normal,pt) + r1;

	if ( penetration < 0.0 )
	{
		return false;
	}

	point = (c1 - r1 * normal);
	return true;
}

bool srCollision::_TouchToUser(Vec3& /*position*/, sr_real /*radius*/, Vec3& /*point*/, Vec3& /*normal*/, sr_real& /*penetration*/)
{
	return false;
}

//////////////////////////////////////////////////////////////////////////
// Sonar Sensor

/* ========================================================================== 
bool _ConeToSlab

description: Detect collision between cone and slab
argument: 
normal: normal vector of slab plane
center: center of slab between both planes
min: displacement to lower plane from center
max: displacement to upper plane from center
T0: frame of cone. origin is apex, x-direction is direction of height of cone
range: height of cone
angle: angle of apex
return:
true for collide
false for not collide
========================================================================== */
bool _ConeToSlab(Vec3& normal, Vec3& center, sr_real min, sr_real max, SE3& T0, sr_real& range, sr_real& radius)
{
	Vec3 Q;
	Vec3 m;

	// position of apex of cone
	Vec3 P(&T0[9]);
	// direction from apex to the center of bottom of cone
	Vec3 direction(&T0[0]);

	m = Cross(normal, direction);
	m = Cross(m, direction);

	// lowest or highest point on the edge of the bottom of cone
	Q = P + range * direction + radius * m;

	P = P - center;
	Q = Q - center;

	// project P onto normal
	sr_real A = Inner(normal, P);
	// project Q onto normal
	sr_real B = Inner(normal, Q);

	if((A > max && B > max) || (A < min && B < min))	// not collide
		return false;

	return true;
}

bool srCollision::_SonarToBox(SE3& T0, sr_real & range , sr_real & angle, bool & res)
{
	// box information
	SE3& Tbox = GetFrame();
	Vec3 dim = 0.5*GetDimension();
	sr_real radius = range * tan(angle * 0.5);

	Vec3 x(1.0, 0.0, 0.0);
	Vec3 y(0.0, 1.0, 0.0);
	Vec3 z(0.0, 0.0, 1.0);
	Vec3 o(0.0);

	// cone's frame w.r.t. box's frame
	SE3 Tcone = Tbox % T0;

	if(    _ConeToSlab(x, o, - dim[0], dim[0], Tcone, range, radius) 
		&& _ConeToSlab(y, o, - dim[1], dim[1], Tcone, range, radius) 
		&& _ConeToSlab(z, o, - dim[2], dim[2], Tcone, range, radius) 
		)	// collide
	{
		res = true;
		return true;
	}

	res = false;
	return false;
}

bool srCollision::_SonarToSphere(SE3& T0, sr_real & range , sr_real & angle, bool & res)
{
	// Sphere
	SE3 & c = GetFrame();
	sr_real r = 0.5*GetDimension()[0];

	// Convert frame w.r.t cone
	SE3 T1 = T0 % c;

	// center of sphere
	Vec3 c_sphere(T1[9], T1[10], T1[11]);

	// center of bottom of cone
	Vec3 c_cone(range, 0.0, 0.0);
	// radius of bottom of cone
	sr_real radius = range * tan(angle * 0.5);

	// Project the center of sphere to the extension of the bottom plane of cone
	Vec3 prjP(range, T1[10], T1[11]);

	// distance between the center of bottom of cone and the center of sphere
	sr_real l = Norm(c_cone - prjP);

	// Find the closest point on the edge of bottom of cone to the projected center of sphere
	Vec3 cp(range, radius * prjP[1] / l, radius * prjP[2] / l);

	// the center of sphere is further than bottom of cone from the apex
	if(T1[9] > range) {
		if(l < radius) {// projected center is inside of bottom
			// then, compare x-direction distance	(from apex to center of bottom is in x-direction)
			if(T1[9] - range <= r) {// collide
				res = true;
				return true;
			}
		}
		else {			// projected center is outside of bottom
			// Calculate distance between the closest point cp and center of sphere
			Vec3 dist = cp - c_sphere;
			if(Norm(dist) <= r) {	// collide
				res = true;
				return true;
			}
		}
	}
	// the center of sphere is located in slab between bottom plane A of cone and the plane parallel to A and including apex.
	else {
		// Find the closest point on the side surface of cone to the center of sphere

		// Find the normal to cp
		Vec3 normal = Cross(c_sphere - c_cone, cp);
		normal = Cross(normal, cp);

		// Line L whose direction is normal and passes the center of sphere
		// Line M which pass the apex and the center of bottom of cone
		// Find the intercept L and M
		sr_real e = - c_sphere[1] / normal[1];
		Vec3 intercept = c_sphere + e * normal;

//		ASSERT(abs(intercept[1]) < 10e-6 && abs(intercept[2]) < 10e-6);
		if(intercept[0] <= 0.0) {	// intercept is above the apex
			if(Norm(c_sphere) <= r) {
				res = true;
				return true;
			}
		}
		else if(intercept[0] > range + radius * radius / range)	{	// intercept 
			if(Norm(c_sphere - prjP) <= r) {
				res = true;
				return true;
			}
		}
		else {
			sr_real s = intercept[0] * sin(angle * 0.5);

			if(Norm(c_sphere - intercept) <= s + r) {
				res = true;
				return true;
			}
		}
	}
	res = false;
	return false;
}

bool srCollision::_SonarToCapsule(SE3& /*T0*/, sr_real & /*range*/, sr_real & /*angle*/, bool & /*res*/)
{
	return false;
}

bool srCollision::_SonarToCylinder(SE3& /*T0*/, sr_real & /*range*/, sr_real & /*angle*/, bool & /*res*/)
{
	return false;
}

bool srCollision::_SonarToPlane(SE3& T0, sr_real & range , sr_real & angle, bool & res)
{
	// Plane
	SE3 & T1 = GetFrame();

	// Cone
	Vec3 position(&T0[9]);
	Vec3 direction(&T0[0]);
	sr_real radius = range * tan(angle * 0.5);
	Vec3 Q;
	Vec3 m;
	Vec3 normal;
	normal.SetValues(T1[6], T1[7], T1[8]);
	Vec3 c(&T1[9]);

	m = Cross(normal, direction);
	m = Cross(m, direction);

	Q = position + range * direction + radius * m;
	Q = Q - c;

	if(Inner(normal, Q) < 0.0) {
		res = true;
		return true;
	}

	res = false;
	return false;
}

bool srCollision::_SonarToUser(SE3& /*T0*/, sr_real & /*range*/, sr_real & /*angle*/, bool & /*res*/)
{
	return false;
}


#include "srgCamera.h"
#include "srgMath.h"
#include "stdio.h"
#include <srConfiguration.h>
const float srgCamera::m_fMinFocusLength = RGL_CAMERA_MIN_FOCUSLENGTH;
const int	srgCamera::m_nApproachRateInv = RGL_CAMERA_APPROACH_INV_RATE;

const Vec3 srgCamera::vZero(0, 0, 0);
//const se3 srgCamera::sse3RotateX(1, 0, 0, 0, 0, 0);
//const se3 srgCamera::sse3RotateY(0, 1, 0, 0, 0, 0);
//const se3 srgCamera::sse3RotateZ(0, 0, 1, 0, 0, 0);
//const se3 srgCamera::sse3TranslateX(0, 0, 0, 1, 0, 0);
//const se3 srgCamera::sse3TranslateY(0, 0, 0, 0, 1, 0);
//const se3 srgCamera::sse3TranslateZ(0, 0, 0, 0, 0, 1);
const se3 srgCamera::sse3RotateX(2, 0, 0, 0, 0, 0);
const se3 srgCamera::sse3RotateY(0, 3, 0, 0, 0, 0);
const se3 srgCamera::sse3RotateZ(0, 0, 4, 0, 0, 0);
const se3 srgCamera::sse3TranslateX(0, 0, 0, 5, 0, 0);
const se3 srgCamera::sse3TranslateY(0, 0, 0, 0, 4, 0);
const se3 srgCamera::sse3TranslateZ(0, 0, 0, 0, 0, 1);

srgCamera::srgCamera():
m_xeProjectionMode(srgCamera::Perspective),
m_dFOVy(45.0),
m_dNear(0.01),
m_dFar(2500.0),
m_AspectRatio(1.0),
m_bLookBack(0),
m_vEye(Vec3(10, 10, 10)),
m_vUp(Vec3(0, 1, 0)),
m_vCenter(vZero)
//m_ProjMat
{
	UpdateCameraFrame();
}

srgCamera::srgCamera(const Vec3& _eye, const Vec3& _center, const Vec3& _up):
m_bLookBack(0),
m_vEye(_eye),
m_vCenter(_center),
m_vUp(_up),
m_xeProjectionMode(srgCamera::Perspective),
m_dFOVy(45.0),
m_dNear(0.01), m_dFar(2500.0),
m_AspectRatio(1.0)
//m_ProjMat
{
	UpdateCameraFrame();
}

srgCamera::~srgCamera(void)
{

}

void srgCamera::Reset(void)
{
	// TODO: default값을 가지고 있을 것.
	m_bLookBack = 0;
	// m_vEye = Vec3(CAMERA_X, CAMERA_Y, CAMERA_Z);
	// m_vCenter = Vec3(0.0, 0.0, 0.6);
	m_vUp = Vec3(0.0, 0.0, 1.0);

  //m_vEye = Vec3(2.253063, 2.011233, 1.341073);
	//m_vCenter = Vec3(0.100610, -0.172796, 0.719789);

    m_vEye = Vec3(4.15287, 2.796113, 1.297027);
	m_vCenter = Vec3(0.92, -0.537, 0.66);


    // Center: 2.228765, 1.549195, 0.276861
    // Eye: 6.331959, -3.582293, 1.531183
	UpdateCameraFrame();
}

inline void srgCamera::UpdateCameraFrame(void)
{
   //printf("Center: %f, %f, %f \n", m_vCenter[0], m_vCenter[1], m_vCenter[2]);
   //printf("Eye: %f, %f, %f \n\n", m_vEye[0], m_vEye[1], m_vEye[2]);

	// __z
	__z = m_vEye - m_vCenter;
	m_dblLength = __z.Normalize();

	// __y
	__y = m_vUp - (Inner(m_vUp, __z)*__z);
	__y.Normalize();	// 2008-04-14

	// __x
	__x = Cross(__y, __z);
	__x.Normalize();

	m_sCameraFrame
		= SE3(
		__x[0], __x[1], __x[2],
		__y[0], __y[1], __y[2],
		__z[0], __z[1], __z[2],
		m_vEye[0], m_vEye[1], m_vEye[2]);
        // for (int i(0); i<3 ; ++i){
        //     printf("x %i: %f \n", i, __x[i]);
        //     printf("y %i: %f \n", i, __y[i]);
        //     printf("z %i: %f \n", i, __z[i]);
        //     printf("m_vEye %i: %f \n", i, m_vEye[i]);
        // }
        // m_sCameraFrame
        //     = SE3(
	// 	0, 1, 0,
	// 	0, 0,1,
	// 	1, 0, 0,
	// 	2, 0, 2);

	m_sCameraInvFrame = Inv(m_sCameraFrame);
}

inline void srgCamera::UpdateCameraVec3s(void)
{
	m_sCameraInvFrame = Inv(m_sCameraFrame);

	m_vUp[0] = m_sCameraFrame[3];
	m_vUp[1] = m_sCameraFrame[4];
	m_vUp[2] = m_sCameraFrame[5];

	m_vEye[0] = m_sCameraFrame[9];
	m_vEye[1] = m_sCameraFrame[10];
	m_vEye[2] = m_sCameraFrame[11];

	m_vCenter[0] = m_vEye[0] - m_dblLength*m_sCameraFrame[6];
	m_vCenter[1] = m_vEye[1] - m_dblLength*m_sCameraFrame[7];
	m_vCenter[2] = m_vEye[2] - m_dblLength*m_sCameraFrame[8];
}

//==============================================================================
// Translate
//==============================================================================
void srgCamera::Translate_World(const Vec3& vMovement_World)
{
	m_vEye += vMovement_World;
	m_vCenter += vMovement_World;

	UpdateCameraFrame();
}
void srgCamera::Translate_World(const Vec3& vUnitAxis_World, float fStep)
{
	m_vEye += fStep*vUnitAxis_World;
	m_vCenter += fStep*vUnitAxis_World;

	UpdateCameraFrame();
}

void srgCamera::Translate_X_World(float fStep)
{
	m_vEye[0] += fStep;
	m_vCenter[0] += fStep;

	UpdateCameraFrame();
}
void srgCamera::Translate_Y_World(float fStep)
{
	m_vEye[1] += fStep;
	m_vCenter[1] += fStep;

	UpdateCameraFrame();
}
void srgCamera::Translate_Z_World(float fStep)
{
	m_vEye[2] += fStep;
	m_vCenter[2] += fStep;

	UpdateCameraFrame();
}
void srgCamera::Translate_Camera(const Vec3& vMovement_Camera)
{
	Vec3 vMovement_World(SE3(m_sCameraFrame.GetOrientation(), vZero)*vMovement_Camera);

	m_vEye += vMovement_World;
	m_vCenter += vMovement_World;

	UpdateCameraFrame();
}
void srgCamera::Translate_Camera(const Vec3& vUnitAxis_Camera, float fStep)
{
	Vec3 vUnitAxis_World(SE3(m_sCameraFrame.GetOrientation(), vZero)*vUnitAxis_Camera);

	m_vEye += fStep*vUnitAxis_World;
	m_vCenter += fStep*vUnitAxis_World;

	UpdateCameraFrame();
}
void srgCamera::Translate_X_Camera(float fStep)
{
	__z = m_vEye - m_vCenter;
	__z.Normalize();
	__x = Cross(m_vUp, __z);
	__x.Normalize();

	m_vEye += fStep*__x;
	m_vCenter += fStep*__x;

	UpdateCameraFrame();
}
void srgCamera::Translate_Y_Camera(float fStep)
{
	m_vEye += fStep*m_vUp;
	m_vCenter += fStep*m_vUp;

	UpdateCameraFrame();
}
void srgCamera::Translate_Z_Camera(float fStep)
{
	__z = m_vEye - m_vCenter;
	__z.Normalize();

	m_vEye += fStep*__z;
	m_vCenter += fStep*__z;

	UpdateCameraFrame();
}

//==============================================================================
// Approach to camera focus point
//==============================================================================
void srgCamera::Approach(float fStep)
{
	// Get camera x-axis and focus length
	__z = (m_vEye - m_vCenter);

	// Approach the camera to the focus point
	// If the camera have to move more than focus length, 
	//the camera would get just minimum focus length that already defined
	if (__z.Normalize() - fStep > m_fMinFocusLength)
		m_vEye -= fStep*__z;
	else
		m_vEye = m_vCenter + m_fMinFocusLength*__z;

	UpdateCameraFrame();
}

void srgCamera::Approach_Smart(int nStep)
{
	if (nStep > 0)
	{
		float ___f = (m_dblLength - m_fMinFocusLength)/float(m_nApproachRateInv);
		if (___f > 0.01f)
			Approach(___f);
		else
			Approach(0.00f);
	}
	if (nStep < 0)
	{
		float ___f = -(m_dblLength - m_fMinFocusLength)/float(m_nApproachRateInv - 1);
		if (___f < -0.01f)
			Approach(___f);
		else
			Approach(-0.01f);
	}
}

//==============================================================================
// Rotate itself
//==============================================================================
void srgCamera::Rotate_World(const Vec3& vUnitAxis_World, float fStep)
{
	__v = m_sCameraFrame.GetPosition();
	m_sCameraFrame
		= Exp(Ad(SE3(__v), se3(vUnitAxis_World[0], vUnitAxis_World[1], vUnitAxis_World[2], 0, 0, 0)),fStep)
		*m_sCameraFrame;
	UpdateCameraVec3s();
}
void srgCamera::Rotate_X_World(float fStep)
{
	__v = m_sCameraFrame.GetPosition();
	m_sCameraFrame
		= Exp(Ad(SE3(__v), sse3RotateX), fStep)
		*m_sCameraFrame;
	UpdateCameraVec3s();
}
void srgCamera::Rotate_Y_World(float fStep)
{
	__v = m_sCameraFrame.GetPosition();
	m_sCameraFrame
		= Exp(Ad(SE3(__v), sse3RotateY), fStep)
		*m_sCameraFrame;
	UpdateCameraVec3s();
}
void srgCamera::Rotate_Z_World(float fStep)
{
	__v = m_sCameraFrame.GetPosition();
	m_sCameraFrame
		= Exp(Ad(SE3(__v), sse3RotateZ), fStep)
		*m_sCameraFrame;
	UpdateCameraVec3s();
}
void srgCamera::Rotate_Camera(const Vec3& vUnitAxis_Camera, float fStep)
{
	m_sCameraFrame
		= Exp(Ad(m_sCameraFrame, se3(vUnitAxis_Camera[0], vUnitAxis_Camera[1], vUnitAxis_Camera[2], 0, 0, 0)), fStep)
		*m_sCameraFrame;
	UpdateCameraVec3s();
}
void srgCamera::Rotate_X_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(m_sCameraFrame, sse3RotateX), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::Rotate_Y_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(m_sCameraFrame, sse3RotateY), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::Rotate_Z_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(m_sCameraFrame, sse3RotateZ), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}

//==============================================================================
// Turn around camera focus and some point that defined by user
//==============================================================================
void srgCamera::TurnCenter_World(const Vec3& vUnitAxis_World, float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_vCenter), se3(vUnitAxis_World[0], vUnitAxis_World[1], vUnitAxis_World[2], 0, 0, 0)), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_X_World(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_vCenter), sse3RotateX), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_Y_World(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_vCenter), sse3RotateY), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_Z_World(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_vCenter), sse3RotateZ), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_Camera(const Vec3& vUnitAxis_Camera, float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_sCameraFrame.GetOrientation(), m_vCenter), se3(vUnitAxis_Camera[0], vUnitAxis_Camera[1], vUnitAxis_Camera[2], 0, 0, 0)), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_X_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_sCameraFrame.GetOrientation(), m_vCenter), sse3RotateX), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_Y_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_sCameraFrame.GetOrientation(), m_vCenter), sse3RotateY), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnCenter_Z_Camera(float fStep)
{
	m_sCameraFrame
		= Exp(Ad(SE3(m_sCameraFrame.GetOrientation(), m_vCenter), sse3RotateZ), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_World(const Vec3& vUnitAxis_World, float fStep, const Vec3& vPoint)
{
	m_sCameraFrame
		= Exp(Ad(SE3(vPoint), se3(vUnitAxis_World[0], vUnitAxis_World[1], vUnitAxis_World[2], 0, 0, 0)), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_X_World(float fStep, const Vec3& vPoint)
{
	m_sCameraFrame
		= Exp(Ad(SE3(vPoint), sse3RotateX), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_Y_World(float fStep, const Vec3& vPoint)
{
	m_sCameraFrame
		= Exp(Ad(SE3(vPoint), sse3RotateY), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_Z_World(float fStep, const Vec3& vPoint)
{
	m_sCameraFrame
		= Exp(Ad(SE3(vPoint), sse3RotateZ), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_Camera(const Vec3& vUnitAxis_Camera, float fStep, const Vec3& vPoint)
{
	__lse3 = m_sCameraFrame;
	__lse3 .SetPosition(vPoint);

	m_sCameraFrame
		= Exp(Ad(__lse3 , se3(vUnitAxis_Camera[0], vUnitAxis_Camera[1], vUnitAxis_Camera[2], 0, 0, 0)), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_X_Camera(float fStep, const Vec3& vPoint)
{
	__lse3 = m_sCameraFrame;
	__lse3 .SetPosition(vPoint);

	m_sCameraFrame
		= Exp(Ad(__lse3 , sse3RotateX), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_Y_Camera(float fStep, const Vec3& vPoint)
{
	__lse3 = m_sCameraFrame;
	__lse3 .SetPosition(vPoint);

	m_sCameraFrame
		= Exp(Ad(__lse3 , sse3RotateY), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}
void srgCamera::TurnPoint_Z_Camera(float fStep, const Vec3& vPoint)
{
	__lse3 = m_sCameraFrame;
	__lse3 .SetPosition(vPoint);

	m_sCameraFrame
		= Exp(Ad(__lse3 , sse3RotateZ), fStep)
		*m_sCameraFrame;

	UpdateCameraVec3s();
}


//==============================================================================
//
void srgCamera::SetCameraFrame(const Vec3& _eye, const Vec3& _center, const Vec3& _up)
{
	m_vEye		= _eye;
	m_vCenter	= _center;
	m_vUp		= _up;

	UpdateCameraFrame();
}

void srgCamera::SetCameraFrame(const SE3& _T, double _length)
{
	m_sCameraFrame = _T;

	// 초점거리(_length)를 지정하지 않았으면 _T의 중심점과 원점까지의 거리를 초점거리로 한다.
	if(_length < 0)
	{
		//Vec3 position(_T[9], _T[10], _T[11]);
		Vec3 position = m_sCameraFrame.GetPosition();
		m_dblLength = Norm(position);
	} else {
		m_dblLength = _length;
	}
	UpdateCameraVec3s();
	m_sCameraInvFrame = Inv(m_sCameraFrame); 
}

SE3 srgCamera::GetCameraFrame()
{
	return m_sCameraFrame;
}
const double* srgCamera::GetGLCameraFrame()
{
	m_sCameraInvFrame.ToArray(m_matCameraInvFrame);
	return m_matCameraInvFrame;
}

Vec3 srgCamera::GetXAxis( void )
{
	return m_sCameraFrame.GetX();
}

Vec3 srgCamera::GetYAxis( void )
{
	return m_sCameraFrame.GetY();
}

Vec3 srgCamera::GetZAxis( void )
{
	return m_sCameraFrame.GetZ();
}

void srgCamera::Translate_ToPoint( Vec3& vPoint, float fLength )
{
	__z = m_vEye - m_vCenter;
	__z.Normalize();
	//__x = Cross(m_vUp, __z);
	//__x.Normalize();

	m_vCenter = vPoint;
	m_vEye = vPoint + fLength*__z;

	UpdateCameraFrame();
}

void srgCamera::SetEye( const Vec3& _Eye )
{
	m_vEye = _Eye;UpdateCameraFrame();
}

void srgCamera::SetCenter( const Vec3& _Center )
{
	m_vCenter = _Center;UpdateCameraFrame();
}

void srgCamera::SetUp( const Vec3& _Up )
{
	m_vUp = _Up;UpdateCameraFrame();
}

Vec3 srgCamera::GetEye( void )
{
	return m_vEye;
}

Vec3 srgCamera::GetEye( const SE3& _CameraFrame )
{
	return _CameraFrame.GetPosition();
}
Vec3 srgCamera::GetUp( void )
{
	return m_vUp;
}

Vec3 srgCamera::GetUp( const SE3& _CameraFrame )
{
	return Vec3(_CameraFrame[3], _CameraFrame[4], _CameraFrame[5]);
}
Vec3 srgCamera::GetCenter( void )
{
	return m_vCenter;
}

Vec3 srgCamera::GetCenter( const SE3& _CameraFrame, double _Length )
{
	return _CameraFrame.GetPosition() - _Length*Vec3(_CameraFrame[6], _CameraFrame[7], _CameraFrame[8]);
}
double srgCamera::GetFocusLength( void )
{
	return m_dblLength;
}

double srgCamera::GetFOVy() const
{
	return m_dFOVy;
}

void srgCamera::SetFOVy( double val )
{
	m_dFOVy = val;
}

double srgCamera::GetNear() const
{
	return m_dNear;
}

void srgCamera::SetNear( double val )
{
	m_dNear = val;
}

double srgCamera::GetFar() const
{
	return m_dFar;
}

void srgCamera::SetFar( double val )
{
	m_dFar = val;
}

double srgCamera::GetAspectRatio() const
{
	return m_AspectRatio;
}

void srgCamera::SetAspectRatio( double val )
{
	m_AspectRatio = val;
}

SE3 srgCamera::GetProjMat() const
{
	return m_ProjMat;
}

void srgCamera::SetProjMat( SE3 val )
{
	m_ProjMat = val;
}

srgCamera::ProjectionType srgCamera::GetProjectionMode() const
{
	return m_xeProjectionMode;
}

void srgCamera::SetProjectionMode( srgCamera::ProjectionType val )
{
	m_xeProjectionMode = val;
}

void srgCamera::SetViewDirection( srgCamera::ViewDirection val )
{
	m_xeViewDirection = val;
}

srgCamera::ViewDirection srgCamera::GetViewDirection() const
{
	return m_xeViewDirection;
}

float srgCamera::GetHeightOfScene( SE3 se3LocalFrame )
{
	return tanf(SRG_DEG2RAD(m_dFOVy)*0.5f)
		*
		// 카메라 frame에서 본 SE3LocalFrame의 원점까지의 벡터 중 Z성분
		// 카메라는 -Z방향을 바라보고 있으므로, 이 값이 카메라 xy평면과 SE3LocalFrame의 원점을 포함하고 평행한 평면까지의
		//값이 된다.
		fabs((Inv(GetCameraFrame())*se3LocalFrame.GetPosition())[2])
		// FOVy는 카메라의 y축방향의 시야각이다. 그런데 앞에서 tan()함수를 위해
		//이 값의 절반을 썼으므로 두 배를 해야 전체 시야에 보이는 평면의 높이가
		//구해진다.
		*2.0f;
}

float srgCamera::GetHeightOfScene( void )
{
	// FOVy는 카메라의 y축방향의 시야각이다. 그런데 앞에서 tan()함수를 위해
	//이 값의 절반을 썼으므로 두 배를 해야 전체 시야에 보이는 평면의 높이가
	//구해진다.
	return tanf(SRG_DEG2RAD(m_dFOVy)*0.5f)*m_dblLength*2.0f;
}

// float srgCamera::GetHeightIntoSight( float fLength_Camera_Plane )
// {
// 	// FOVy는 카메라의 y축방향의 시야각이다. 그런데 앞에서 tan()함수를 위해
// 	//이 값의 절반을 썼으므로 두 배를 해야 전체 시야에 보이는 평면의 높이가
// 	//구해진다.
// 	return tanf(SRG_DEG2RAD(m_dFOVy)*0.5f)*fLength_Camera_Plane*2.0f;
// }

void srgCamera::Translate_View( int FromX, int FromY, int ToX, int ToY, float ViewSizeY )
{
	double Rate = GetHeightOfScene()/ViewSizeY;

	Translate_Camera(Vec3(Rate*(FromX - ToX), Rate*(FromY - ToY), 0.0f));
}

void srgCamera::Turn_Focus( int FromX, int FromY, int ToX, int ToY, int ViewportX, int ViewportY, int ViewportSizeX, int ViewportSizeY, float fRadius_Circle_ViewPort )
{
	int nCenterX = ViewportX + ViewportSizeX*0.5f;
	int nCenterY = ViewportY + ViewportSizeY*0.5f;

	FromX -= nCenterX;
	FromY -= nCenterY;
	ToX -= nCenterX;
	ToY -= nCenterY;

	float fX1_Sphere = FromX/fRadius_Circle_ViewPort;
	float fY1_Sphere = FromY/fRadius_Circle_ViewPort;

	float fX2_Sphere = ToX/fRadius_Circle_ViewPort;
	float fY2_Sphere = ToY/fRadius_Circle_ViewPort;

	float fNorm1 = fabs(sqrtf(fX1_Sphere*fX1_Sphere + fY1_Sphere*fY1_Sphere));
	float fNorm2 = fabs(sqrtf(fX2_Sphere*fX2_Sphere + fY2_Sphere*fY2_Sphere));;

	if (fNorm2 < 1.0f && fNorm1 < 1.0f)
	{
		//float fZ1_Sphere = fabs(sqrtf(1 - fX1_Sphere*fX1_Sphere - fY1_Sphere*fY1_Sphere));
		//float fZ2_Sphere = fabs(sqrtf(1 - fX2_Sphere*fX2_Sphere - fY2_Sphere*fY2_Sphere));

		Vec3 v1(
			fX1_Sphere,
			fY1_Sphere,
			fabs(sqrtf(1 - fX1_Sphere*fX1_Sphere - fY1_Sphere*fY1_Sphere))
			);
		Vec3 v2(
			fX2_Sphere,
			fY2_Sphere,
			fabs(sqrtf(1 - fX2_Sphere*fX2_Sphere - fY2_Sphere*fY2_Sphere))
			);

		Vec3 vRotationAxis = Cross(v2, v1);
		vRotationAxis.Normalize();
		float fNorm = (v2 - v1).Normalize();

		TurnCenter_Camera(vRotationAxis, acosf(1.0f - 0.5*fNorm*fNorm));
	}
	else if (fNorm2 == 1.0f || fNorm1 == 1.0f)
	{
		return;
	}
	else
	{
		float fNormalAngle = (fNorm1 - fNorm2)*0.25f*M_PI;

		float fAngle1;
		float fAngle2;

		if (fY1_Sphere > 0)			fAngle1 = acosf(fX1_Sphere/fNorm1);
		else if (fY1_Sphere < 0)	fAngle1 = -acosf(fX1_Sphere/fNorm1);
		else						fAngle1 = 0.0f;

		if (fY2_Sphere > 0)			fAngle2 = acosf(fX2_Sphere/fNorm2);
		else if (fY2_Sphere < 0)	fAngle2 = -acosf(fX2_Sphere/fNorm2);
		else						fAngle2 = 0.0f;		

		Vec3 vRotationAxis(-fNormalAngle*fY1_Sphere, fNormalAngle*fX1_Sphere, fAngle1 - fAngle2);
		float fAngleSum = vRotationAxis.Normalize();

		TurnCenter_Camera(vRotationAxis, fAngleSum);		
	}
}

//srgCameraMemory::srgCameraMemory( /*const CString& CamName*/ /*= _T("Noname") */ )
//{
//
//}
//
//srgCameraMemory::~srgCameraMemory()
//{
//
//}
//

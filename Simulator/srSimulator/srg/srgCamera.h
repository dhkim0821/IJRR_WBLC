/*******************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2007-3-22          Jeongseok Lee
v0.004 : 2008-11-12			Jeongseok Lee
v0.005 : 2009-01-23			Jeongseok Lee
*******************************************************************************/

#ifndef __SRG_CAMERA__
#define __SRG_CAMERA__

#include "srgL.h"
#include "srgLieGroup.h"
#include <iostream>
#include <list>
#include <algorithm>

#define RGL_CAMERA_MIN_FOCUSLENGTH	(0.001)
#define RGL_CAMERA_APPROACH_INV_RATE	(25)


/*!
	\class srgCamera
	\brief A class of Camera for OpenGL.

	This class has the status informations of Camera,i.e. Viewing volume,
	that are the rotation, position and focus length.
	The status is changed by user's input like mouse cursor,
	SE3, etc. The concept of xrCamera class is based on
	gluLookAt function of OpenGL. 
*/
class srgCamera
{
public:
	/*! A Constructor
	*  @param _eye eye vector of camera
	*  @param _center center vector of camera
	*  @param _up up vector of camera
	*/
	srgCamera();
	srgCamera(const 	Vec3& _eye,
		const Vec3& _center,
		const Vec3& _up);
	virtual ~srgCamera(void);

	/*!
		Reset camera
	*/
	void Reset(void);

	enum ViewDirection
	{
		Top, Free, Front, Left
	};

	enum ProjectionType
	{
		Orthogonal, Perspective
	};

	ViewDirection	m_xeViewDirection;
	ProjectionType	m_xeProjectionMode;
	double			m_dFOVy;
	double			m_dNear, m_dFar;
	double			m_AspectRatio;
	SE3				m_ProjMat;

	double			GetFOVy() const;
	void			SetFOVy(double val);
	double			GetNear() const;
	void			SetNear(double val);
	double			GetFar() const;
	void			SetFar(double val);
	double			GetAspectRatio() const;
	void			SetAspectRatio(double val);
	SE3				GetProjMat() const;
	void			SetProjMat(SE3 val);
	ViewDirection	GetViewDirection() const;
	void			SetViewDirection(ViewDirection val);
	ProjectionType	GetProjectionMode() const;
	void			SetProjectionMode(ProjectionType val);

	SE3			GetCameraFrame			(void);
	void		SetCameraFrame			(const Vec3& _eye, const Vec3& _center, const Vec3& _up);
	void		SetCameraFrame			(const SE3& _T, double _length = -1.0);
	const double*	GetGLCameraFrame		(void);

	// Translate
	void	Translate_World(const Vec3& vMovement_World);
	void	Translate_World(const Vec3& vUnitAxis_World, float fStep);
	void	Translate_X_World(float fStep);
	void	Translate_Y_World(float fStep);
	void	Translate_Z_World(float fStep);
	void	Translate_Camera(const Vec3& vMovement_Camera);
	void	Translate_Camera(const Vec3& vUnitAxis_Camera, float fStep);
	void	Translate_X_Camera(float fStep);
	void	Translate_Y_Camera(float fStep);
	void	Translate_Z_Camera(float fStep);
	void	Translate_View(int FromX, int FromY, int ToX, int ToY, float ViewSizeY);
	void	Translate_ToPoint(Vec3& vPoint, float fLength);

	// Zoom
	static const float	m_fMinFocusLength;
	static const int	m_nApproachRateInv;
	bool	m_bLookBack;
	void	Zoom(double _fStep);
	void	Approach(float fStep);
	void	Approach_Smart(int Sign);

	// Rotate
	void	Rotate_World(const Vec3& vUnitAxis_World, float fStep);
	void	Rotate_X_World(float fStep);
	void	Rotate_Y_World(float fStep);
	void	Rotate_Z_World(float fStep);
	void	Rotate_Camera(const Vec3& vUnitAxis_Camera, float fStep);
	void	Rotate_X_Camera(float fStep);
	void	Rotate_Y_Camera(float fStep);
	void	Rotate_Z_Camera(float fStep);
	void	TurnCenter_World(const Vec3& vUnitAxis_World, float fStep);
	void	TurnCenter_X_World(float fStep);
	void	TurnCenter_Y_World(float fStep);
	void	TurnCenter_Z_World(float fStep);
	void	TurnCenter_Camera(const Vec3& vUnitAxis_Camera, float fStep);
	void	TurnCenter_X_Camera(float fStep);
	void	TurnCenter_Y_Camera(float fStep);
	void	TurnCenter_Z_Camera(float fStep);
	void	TurnPoint_World(const Vec3& vUnitAxis_World, float fStep, const Vec3& vPoint);
	void	TurnPoint_X_World(float fStep, const Vec3& vPoint);
	void	TurnPoint_Y_World(float fStep, const Vec3& vPoint);
	void	TurnPoint_Z_World(float fStep, const Vec3& vPoint);
	void	TurnPoint_Camera(const Vec3& vUnitAxis_Camera, float fStep, const Vec3& vPoint);
	void	TurnPoint_X_Camera(float fStep, const Vec3& vPoint);
	void	TurnPoint_Y_Camera(float fStep, const Vec3& vPoint);
	void	TurnPoint_Z_Camera(float fStep, const Vec3& vPoint);
	void	Turn_Focus(
		int FromX, int FromY, int ToX, int ToY,
		int ViewportX, int ViewportY,
		int ViewportSizeX, int ViewportSizeY,
		float fRadius_Circle_ViewPort
		);

public:
	Vec3				m_vEye;
	Vec3				m_vUp;
	Vec3				m_vCenter;
	double				m_dblLength;
	Vec3				GetEye(void);
	Vec3				GetUp(void);
	Vec3				GetCenter(void);
	double				GetFocusLength(void);

	Vec3				GetXAxis( void );
	Vec3				GetYAxis( void );
	Vec3				GetZAxis( void );

	void				SetEye(const Vec3& _Eye);
	void				SetUp(const Vec3& _Up);
	void				SetCenter(const Vec3& _Center);

	SE3					m_sCameraFrame;
	SE3					m_sCameraInvFrame;
	double			m_matCameraInvFrame[16];

	void	UpdateCameraFrame(void);
	void	UpdateCameraVec3s(void);

	// Helper functions
	Vec3	GetCenter	(const SE3& _CameraFrame, double _Length);
	Vec3	GetEye		(const SE3& _CameraFrame);
	Vec3	GetUp		(const SE3& _CameraFrame);
	
	float GetHeightOfScene(void);
	float GetHeightOfScene(SE3 se3LocalFrame);
private:
	// for performance
	Vec3 __x, __y, __z;
	Vec3 __v;
	SE3 __lse3;
	se3 __sse3;
	static const Vec3 vZero;
	static const se3 sse3RotateX;
	static const se3 sse3RotateY;
	static const se3 sse3RotateZ;
	static const se3 sse3TranslateX;
	static const se3 sse3TranslateY;
	static const se3 sse3TranslateZ;

	//srgCameraMemory	m_DefCamMem;
	//friend class srgCameraMemory;
};



#endif // __SRG_CAMERA__

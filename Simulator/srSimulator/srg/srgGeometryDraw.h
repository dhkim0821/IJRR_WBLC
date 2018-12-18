/*******************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.100 : 2008-11-11			Jeongseok Lee
*******************************************************************************/

#ifndef __SRG_GEOMETRY_DRAW__
#define __SRG_GEOMETRY_DRAW__

#include "srgL.h"
#include "srgLieGroup.h"
#include "srgMath.h"
#include <string>

// naming policy: (srg)(GeometryName)(_Options)
// srg : prefix
// Geometry : 
// _Options : 
//     FeatureLine : 
//     NV : with normal vector
//     TEX : with texture coordinate

#define LINE_STIPPLE_FACTOR		1
#define LINE_STIPPLE_PATTER		0x00FF

// NOTE
#define SRG_FIGURE_DRAWINGSTYLE_MASK	0x000000FF
#define SRG_FIGURE_FILL				0x00000001
//#define SRG_FIGURE_LINE				0x00000002
#define SRG_FIGURE_SILHOUETTE		0x00000003
//#define SRG_FIGURE_EDGE				0x00000004
#define SRG_FIGURE_BACK_FEATURELINE	0x00000005
#define SRG_FIGURE_WIREFRAME			0x00000006
#define SRG_FIGURE_FEATURELINE		0x00000007

#define SRG_FIGURE_TEXTURE_MASK		0x00000F00
#define SRG_FIGURE_TEXTURE			0x00000100
//#define SRG_FIGURE_NOTEXTURE			0x00000020

enum srgDrawStyle
{
	drawstyleFILL			= 0,	// Volume
	drawstyleLINE			= 1,	// Wire frame
	drawstyleSILHOUETTE		= 2,	// Out-line
	drawstyleEDGE			= 3,	// Feature line
	drawstyleBEHIND_EDGE	= 4		// Back feature line
};

struct PointState
{
	//bool	bHasColor;
	//GLfloat	Color[4];
	//	xrColor	clrMaterialColor;
};

struct LineState 
{
	//bool	bHasColor;
	//GLfloat	Color[4];
	//	xrColor	clrMaterialColor;
};

struct QuadricState
{
	srgDrawStyle		DrawStyle;
	bool				HasTexture;
};

//////////////////////////////////////////////////////////////////////////
// BOX
void srgDrawBox_FeatureLine(double Half_X, double Half_Y, double Half_Z);
void srgDrawBox(double Half_X, double Half_Y, double Half_Z);
void srgDrawBox_NV(double Half_X, double Half_Y, double Half_Z);
void srgDrawBox_NV_TEX(double Half_X, double Half_Y, double Half_Z);

//////////////////////////////////////////////////////////////////////////
// OPEN DOME
void srgDrawOpenDome_FeatureLine(double Radius, int FeatureSlices, int FeatureStacks, int Slices, int Stacks);
void srgDrawOpenDome(double Radius, int Slices, int Stacks);
void srgDrawOpenDome_NV(double Radius, int Slices, int Stacks);
void srgDrawOpenDome_NV_TEX(double Radius, int Slices, int Stacks);

//////////////////////////////////////////////////////////////////////////
// CYLINDER
void srgDrawOpenCylinder_FeatureLine(double BaseRadius, double TopRadius, double Height, int FeatureSlices, int Slices);
void srgDrawSphere_FeatureLine(double Radius, int FeatureSlices, int FeatureStacks, int Slices, int Stacks);
void srgDrawLoop(float Radius, int Slices);
void srgDrawCircle(float Radius, int Slices);
void srgDraw2DSphere(float Radius, int Slices, int Stacks);
void srgDrawArrow1(float Length);
void srgDrawTorus(GLfloat majorRadius, GLfloat minorRadius, GLint numMajor, GLint numMinor);

void ConvertXrQuadricToOpenglQuadric(QuadricState QuadricState, GLUquadricObj* obj);
void ConvertXrQuadricToOpenglQuadric(DWORD dwFlags, GLUquadricObj* obj);

void srgDrawPoint(PointState ePointState);
void srgDrawLine(LineState eLineState, double dLength);
void srgDrawLine(float fX1, float fY1, float fZ1, float fX2, float fY2, float fZ2);
void srgDrawFrustumLine(float fFovy, float fAspect, float fzNear, float fzFar);
void srgDrawDisk(double InnerRadius, double OuterRadius, int Slices, int Loops, DWORD dwFlag);
void srgDrawDisk(QuadricState eQuadricState, double InnerRadius, double OuterRadius, int Slices, int Loops);
void srgDrawDisk(double InnerRadius, double OuterRadius, int Slices, int Loops);
void srgPartialDrawDisk(QuadricState eQuadricState, double InnerRadius, double OuterRadius, int Slices, int Loops, double dStartAngle, double dSweepAngle);
void srgDrawSphere(QuadricState eQuadricState, double Radius, int Slices, int Stacks);
void srgDrawSphere(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawOpenDome(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawOpenDome(QuadricState eQuadricState, double Radius, int Slices, int Stacks);
void srgDrawDome(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawDome(QuadricState eQuadricState, double Radius, int Slices, int Stacks);
void srgDrawOpenCylinder(DWORD dwFlags, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawOpenCylinder(QuadricState eQuadricState, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawCylinder(DWORD dwFlags, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawCylinder(QuadricState eQuadricState, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawCapsule(DWORD dwFlags, double Radius, double Height, int Slices, int Stacks);
void srgDrawCapsule(QuadricState	eQuadricState, double Radius, double Height, int Slices, int Stacks);
void srgDrawCapsule(DWORD dwFlags, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawCapsule(QuadricState	eQuadricState, double BaseRadius, double TopRadius, double Height, int Slices, int Stacks);
void srgDrawPartialDisk(QuadricState	eQuadricState, double InnerRadius, double OuterRadius, int Slices, int Loops, double startAngle, double SweepAngle);
void srgDrawBox(QuadricState	eQuadricState, double X, double Y, double Z);
void srgDrawBox(DWORD dwFlags, double X, double Y, double Z);

////////////////////////////////////////////////////////////////////////////
///* For UI */
void srgDrawGrid(void);
void srgDrawGrid_Flexible(int ViewportHeight, float SightHeight);
void srgDrawCameraFocus(void);
void srgDrawCoordinate_axis_basic(void);
void srgDrawCoordinate_axis_basic(float size);

// enum JOINTTYPE { REVOLUTE, PRISMATIC, UNIVERSAL, WELD };
void srgDrawRevoluteJoint(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawPrismaticJoint(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawUniversalJoint(DWORD dwFlags, double Radius, int Slices, int Stacks);
void srgDrawWeldJoint(DWORD dwFlags, double Radius, int Slices, int Stacks);

#endif // __SRG_GEOMETRY_DRAW__

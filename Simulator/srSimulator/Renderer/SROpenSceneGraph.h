/*!*****************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee (jslee02@gmail.com)

Version information
v0.001 : 2009-04-15			Jeongseok Lee
*******************************************************************************/

#ifndef __SR_OPEN_SCENE_GRAPH__
#define __SR_OPEN_SCENE_GRAPH__

#include "common/utils.h"
#include "srDyn/srSpace.h"


#include <osg/NodeCallback>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgDB/DatabasePager>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ShadowTexture>
#include <osg/LightSource>

#include <string>
#include <vector>

// Needs GLUT
// Windows
#ifdef WIN32
	#define GLUT_DISABLE_ATEXIT_HACK
	#include <gl/GLut.h>
	#include <osgViewer/api/win32/GraphicsWindowWin32>
#endif

// Mac OS X
#ifdef __APPLE__
	#include <GLUT/glut.h>
#endif

// LINUX
#ifdef linux
	#include <GL/glut.h>
#endif

//#define SG_NOTHING		0x0000000000000000
//#define SG_NOTHING		0x0000000000000000
//#define SG_NOTHING		0x0000000000000000
//#define SG_NOTHING		0x0000000000000000
//#define SG_NOTHING		0x0000000000000000

class XGUtil
{
public:
	static inline void ChangeColor3(osg::ShapeDrawable* pShapeDrawable, const Vec3& vColor)
	{
		pShapeDrawable->setColor(osg::Vec4(vColor[0], vColor[1], vColor[2], (pShapeDrawable->getColor())[3]));
	}

	static inline void ChangeColor3UInt(osg::ShapeDrawable* pShapeDrawable, const Vec3& vColor)
	{
		pShapeDrawable->setColor(osg::Vec4(vColor[0]/256.0, vColor[1]/256.0, vColor[2]/256.0, (pShapeDrawable->getColor())[3]));
	}

	static inline void ChangeColor4(osg::ShapeDrawable* pShapeDrawable, const osg::Vec4& vColor)
	{
		pShapeDrawable->setColor(osg::Vec4(vColor[0], vColor[1], vColor[2], vColor[3]));
	}

	static inline void ChangeColor4UInt(osg::ShapeDrawable* pShapeDrawable, const osg::Vec4& vColor)
	{
		pShapeDrawable->setColor(osg::Vec4(vColor[0]/256.0, vColor[1]/256.0, vColor[2]/256.0, vColor[3]/256.0));
	}

protected:
private:
};

/*!
\class srgEntity
\brief Base class of graphic object.
*/
class srgEntity : public osg::MatrixTransform
{
public:
};

/*!
\class srgLink
\brief Graphic object which has rendering information for link.
*/
class srgLink : public srgEntity
{
public:
	srgLink(srLink* a_pLink, osg::Object::DataVariance a_DataVariance = osg::Object::DYNAMIC);
	~srgLink();

	srLink*			m_pLink;
	void UpdateLocalFrame(void);
};

class TransformCB : public osg::NodeCallback
{
public:
	TransformCB() {};

	virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv );
protected:
};

///*!
//\class srgJoint
//\brief Graphic object which has rendering information for joint.
//*/
//class srgJoint : public srgEntity
//{
//public:
//	srJoint*		m_pJoint;
//	virtual void	DynamicRender(void* pVoid);
//};
//
///*!
//\class srgCollision
//\brief Graphic object which has rendering information for collision.
//*/
//class srgCollision : public srgEntity
//{
//public:
//	srCollision*	m_pCollision;
//	virtual void	DynamicRender(void* pVoid);
//};
//
///*!
//\class srgSensor
//\brief Graphic object which has rendering information for sensor.
//*/
//class srgSensor : public srgEntity
//{
//public:
//	srSensor*		m_pSensor;
//	virtual void	DynamicRender(void* pVoid);
//};

/*!	\brief SRSceneGraph
*/
class SRSceneGraph
{
public:
	SRSceneGraph() {};
	~SRSceneGraph() {};

	void Init();
	osg::ref_ptr<osg::Group> GetModel();
	void CreateRenderingWorld( srSpace* pxSpace );
protected:
	

	//bool m_Done;

	//HWND m_hWnd;
	//osgViewer::Viewer* mViewer;

	osg::ref_ptr<osg::Group> m_Root;
	osg::ref_ptr<osgShadow::ShadowedScene> m_ShadowedScene;

	//osg::ref_ptr<osg::Node> m_Model;
	//osg::ref_ptr<osgGA::TrackballManipulator> m_Trackball;
	//osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> m_KeyswitchManipulator;

	/// Rendering entities
	// Links
	std::vector< osg::ref_ptr<srgLink> > m_pxgLinks;
};

#endif // __SR_OPEN_SCENE_GRAPH__


/*!*****************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee (jslee02@gmail.com)

Version information
v0.001 : 2009-04-15			Jeongseok Lee
*******************************************************************************/

#include "SROpenSceneGraph.h"

//#include "../Engine/XEngine.h"

#include "srDyn/srSpace.h"
#include "srDyn/srSystem.h"
#include "srDyn/srLink.h"
#include "srDyn/srJoint.h"
#include "srDyn/srRevoluteJoint.h"

#include <osg/Geometry>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>


void SRSceneGraph::CreateRenderingWorld( srSpace* pxSpace )
{
	if (pxSpace == NULL) return;

	// Clear all
	// TODO: 트리에 링크말고 다른 것이 올 경우를 대비하자.
	m_Root->removeChildren(0, m_Root->getNumChildren());
	m_pxgLinks.clear();

	//////////////////////////////////////////////////////////////////////////
	// Ground (임시임)
	////////////////// ////////////////////////////////////////////////////////
	/// Shape (Sphere)
	osg::ref_ptr<osg::Box> BoxForGround( new osg::Box(osg::Vec3(0.0, 0.0, -0.5), 100, 100, 1));

	/// Drawable (ShapeDrawable)
	osg::ref_ptr<osg::ShapeDrawable> GroundSD( new osg::ShapeDrawable( BoxForGround.get() ) );
	// ..Color
	XGUtil::ChangeColor3(GroundSD.get(), Vec3(0.8, 0.8, 0.8));

	/// Geode
	osg::ref_ptr<osg::Geode> GroundGeode( new osg::Geode );
	GroundGeode->addDrawable( GroundSD.get() );

	/// MatrixTransform:
	osg::ref_ptr<osg::MatrixTransform> Ground (new osg::MatrixTransform);
	Ground->addChild(GroundGeode.get());

	osg::Matrix Mat;
	//Mat.makeRotate(DEG2RAD(90), osg::Vec3(1.0, 0.0, 0.0));

	Ground->setMatrix(Mat);

	/// Root
	//m_Joints.push_back(pNewGJoint.get());
	m_Root->addChild(Ground.get());

	// Create space for drawing
	//CreateRenderingWorld(pxSpace);

	// Link
	srSystem* pxSystemItr = NULL;
	srLink* pxLinkItr = NULL;
	srGeometryInfo* pxGeomInfoItr = NULL;
	int NumSystems, NumLinks;

	NumSystems = pxSpace->m_Systems.get_size();
	for (int i = NumSystems - 1; i > -1; --i)
	{
		pxSystemItr = pxSpace->m_Systems.get_at(i);
		NumLinks = pxSystemItr->m_KIN_Links.get_size();

		for (int j = NumLinks - 1; j > -1; --j)
		{
			//////////////////////////////////////////////////////////////////////////
			// Links
			//////////////////////////////////////////////////////////////////////////
			/// pre-step: (get some information for draw)
			// Link pointer
			pxLinkItr = pxSystemItr->m_KIN_Links.get_at(j);
			// Geometry information of Link: Dimension, Color
			real x, y, z;
			float r, g, b, a;
			pxGeomInfoItr = &(pxLinkItr->GetGeomInfo());
			pxGeomInfoItr->GetDimension(x, y, z);
			pxGeomInfoItr->GetColor(r, g, b, a);

			/// Shape
			osg::ref_ptr<osg::Shape> LinkShape = NULL;
			switch (pxGeomInfoItr->GetShape())
			{
			case srGeometryInfo::BOX:
				LinkShape = new osg::Box(osg::Vec3(0.0, 0.0, 0.0), x, y, z);
				break;
			case srGeometryInfo::CAPSULE:
				LinkShape = new osg::Capsule(osg::Vec3(0.0, 0.0, 0.0), 0.5*x, y);
				break;
			case srGeometryInfo::CYLINDER:
				LinkShape = new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.0), 0.5*x, y);
				break;
			case srGeometryInfo::SPHERE:
				LinkShape = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 0.5*x);
				break;
			case srGeometryInfo::PLANE:
//				LinkShape = new osg::InfinitePlane;
//				break;
			case srGeometryInfo::USER:
				break;
			}

			/// Drawable
			osg::ref_ptr<osg::ShapeDrawable> LinkDrawable( new osg::ShapeDrawable( LinkShape.get() ) );
			// ..Color
			XGUtil::ChangeColor4(LinkDrawable.get(), osg::Vec4(r, g, b, a));
			LinkDrawable->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			
			/// Geode
			osg::ref_ptr<osg::Geode> LinkGeode( new osg::Geode );
			LinkGeode->addDrawable( LinkDrawable.get() );

			/// MatrixTransform
			osg::ref_ptr<srgLink> pxgNewLink = new srgLink(pxLinkItr);
			// Set data variance to DYNAMIC to let OSG know that we will
			//   modify this node during the update traversal.
			pxgNewLink->setDataVariance(osg::Object::DYNAMIC);
			// Set the update callback.
			pxgNewLink->setUpdateCallback(new TransformCB);
			pxgNewLink->addChild(LinkGeode.get());

			/// Add to Root
			m_Root->addChild(pxgNewLink.get());
			m_pxgLinks.push_back(pxgNewLink.get());
		}
	}
}

void SRSceneGraph::Init()
{
	// Init the main Root Node/Group
	m_Root = new osg::Group;
	m_ShadowedScene = new osgShadow::ShadowedScene;

	//m_Root->setDataVariance(osg::Object::STATIC);

	//Shadow stuff!!!
	osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::SoftShadowMap;
	m_ShadowedScene->setShadowTechnique(sm.get());

	//Main light source 
	osg::Vec3 lightPosition(3, 2, 3); 
	osg::LightSource* ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(lightPosition, 0));
	ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1.0));
	ls->getLight()->setDiffuse(osg::Vec4(0.6,0.6,0.6,1.0));

	m_ShadowedScene->addChild(m_Root.get());
	m_ShadowedScene->addChild(ls);

	// Create space for drawing
	//CreateRenderingWorld(pxSpace);
}

osg::ref_ptr<osg::Group> SRSceneGraph::GetModel()
{
	return m_ShadowedScene.get();
}
srgLink::srgLink( srLink* a_pLink, osg::Object::DataVariance a_DataVariance /*= osg::Object::DYNAMIC*/ ) : m_pLink(a_pLink)
{
	setDataVariance(a_DataVariance);
}

srgLink::~srgLink()
{

}

void srgLink::UpdateLocalFrame( void )
{
	real ArrMat[16];
	m_pLink->GetFrame().ToArray(ArrMat);

	osg::Matrix Mat(
		ArrMat[0], ArrMat[1], ArrMat[2], ArrMat[3], 
		ArrMat[4], ArrMat[5], ArrMat[6], ArrMat[7], 
		ArrMat[8], ArrMat[9], ArrMat[10], ArrMat[11], 
		ArrMat[12], ArrMat[13], ArrMat[14], ArrMat[15]
	);

	setMatrix(Mat);
}


void TransformCB::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	srgLink* mtLeft = dynamic_cast<srgLink*>( node );
	mtLeft->UpdateLocalFrame();

	// Continue traversing so that OSG can process any other nodes with callbacks.
	traverse( node, nv );
}

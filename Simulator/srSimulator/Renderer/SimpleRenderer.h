/*!*****************************************************************************
  [ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-11			Jeongseok Lee
v0.005 : 2009-01-23			Jeongseok Lee
 *******************************************************************************/

#ifndef __SIMPLE_RENDERER__
#define __SIMPLE_RENDERER__

#include "srDyn/srSpace.h"

#include "common/utils.h"

#include "srg/srg.h"
#include "srg/srgL.h"
#include "srg/srgColor.h"
#include "srg/srgCamera.h"
#include "srg/srgFont.h"
#include "srg/srgGeometryDraw.h"

#include "Renderer/Model3DS.h"
#include "Renderer/ModelMesh.h"

/*!
  \class srgEntity
  \brief Base class of graphic object.
  */
class srgEntity
{
    public:
        bool			m_bDLCompiled;
        unsigned int	m_iDL_RenderingMode;
        unsigned int	m_iDL_DesignMode;
        //bool			m_bTextureMatrix;
        //virtual void	StaticRender(void* pVoid) = 0;
        virtual void	DynamicRender(void* pVoid) = 0;

        Model3DS*		m_Model;
        ModelMesh*   m_Mesh_Model;

        srgEntity() {
            m_Model = NULL;
            m_Mesh_Model = NULL;
        };
        ~srgEntity() {
            if(m_Model != NULL)  delete m_Model;
            if(m_Mesh_Model != NULL)  delete m_Mesh_Model;
        };
        Model3DS* Load3DSModel(char* name, SE3 T = SE3(0.0)) {
            m_Model = new Model3DS;
            T.ToArray(m_Model->_T);
            m_Model->Load(name);
            return m_Model;
        };
        ModelMesh* LoadMeshModel(char* name, Vec3 mesh_scale, SE3 T = SE3(0.0)) {
            m_Mesh_Model = new ModelMesh;
            m_Mesh_Model->LocalFrame = T;
            T.ToArray(m_Mesh_Model->_T);
            m_Mesh_Model->mesh_scale_ = mesh_scale;
            m_Mesh_Model->Load(name);
            return m_Mesh_Model;
        };

};

/*!
  \class srgLink
  \brief Graphic object which has rendering information for link.
  */
class srgLink : public srgEntity
{
    public:
        struct DYNAMIC_RENDER_DATA 
        {
            int		ViewportHeight;
            float	SightHeight;
        };

        srLink*			m_pLink;
        virtual void	DynamicRender(void* pVoid);
};

/*!
  \class srgJoint
  \brief Graphic object which has rendering information for joint.
  */
class srgJoint : public srgEntity
{
    public:
        struct DYNAMIC_RENDER_DATA 
        {
            int		ViewportHeight;
            float	SightHeight;
        };

        srJoint*		m_pJoint;
        virtual void	DynamicRender(void* pVoid);
};

/*!
  \class srgCollision
  \brief Graphic object which has rendering information for collision.
  */
class srgCollision : public srgEntity
{
    public:
        srCollision*	m_pCollision;
        virtual void	DynamicRender(void* pVoid);
};

/*!
  \class srgSensor
  \brief Graphic object which has rendering information for sensor.
  */
class srgSensor : public srgEntity
{
    public:
        srSensor*		m_pSensor;
        virtual void	DynamicRender(void* pVoid);
};

/*!
  \class srSimpleRenderer
  \brief Renderer for space.

  srSimpleRenderer will render your space and some information related with simulation.
  */
class srSimpleRenderer
{
    public:
        /*!	Constructor. */
        srSimpleRenderer();

        /*! Destructor. */
        ~srSimpleRenderer();

        // Callback function prototype
        typedef void(*SRSR_PFNDELEGATE)(void*);

        /*!	STEP1: Initialize.
         * This member function must run first. */
        void Init(void);

        /*!	STEP2: Set target to render. */
        void SetTarget(const srSpace* pSpace);

        /*! STEP3: Set camera to use. */
        void SetCamera(srgCamera* pCamera);



        /*!	Render scene once. */
        void RenderSceneOnce(void);

        /*!	Update window size. */
        void UpdateWindowSize(int w, int h);



        /*!	Get view port x. */
        const int GetViewportX() const;

        /*!	Get view port y. */
        const int GetViewportY() const;

        /*!	Get width of view port. */
        const int GetViewportW() const;

        /*!	Get height of view port. */
        const int GetViewportH() const;



        /*!	Get status of Space showing. */
        bool IsSpaceVisible() const { return _bShowSpace; }

        /*!	Show Space. */
        void ShowSpace(void) { _bShowSpace = true; }

        /*!	Hide Space. */
        void HideSpace(void) { _bShowSpace = false; }



        /*!	Get status of Link showing. */
        bool IsLinkVisible() const { return _bShowLink; }

        /*!	Show Links. */
        void ShowLink(void) { _bShowLink = true; }

        /*!	Hide Links. */
        void HideLink(void) { _bShowLink = false; }



        /*!	Get status of Joint showing. */
        bool IsJointVisible() const { return _bShowJoint; }

        /*!	Show Joints. */
        void ShowJoint(void) { _bShowJoint = true; }

        /*!	Hide Joints. */
        void HideJoint(void) { _bShowJoint = false; }



        /*!	Get status of Collision showing. */
        bool IsCollisionVisible() const { return _bShowCollision; }

        /*!	Show Collision. */
        void ShowCollision(void) { _bShowCollision = true; }

        /*!	Hide Collision. */
        void HideCollision(void) { _bShowCollision = false; }



        /*!	Get status of Sensor showing. */
        bool IsSensorVisible() const { return _bShowSensor; }

        /*!	Show Sensors. */
        void ShowSensor(void) { _bShowSensor = true; }

        /*!	Hide Sensors. */
        void HideSensor(void) { _bShowSensor = false; }







        /*!	Get status of LinkCoordinate showing. */
        bool IsLinkCoordinateVisible() const { return _bShowLinkCoordinate; }

        /*!	Show LinkCoordinates. */
        void ShowLinkCoordinate(void) { _bShowLinkCoordinate = true; }

        /*!	Hide LinkCoordinates. */
        void HideLinkCoordinate(void) { _bShowLinkCoordinate = false; }



        /*!	Get status of JointCoordinate showing. */
        bool IsJointCoordinateVisible() const { return _bShowJointCoordinate; }

        /*!	Show JointCoordinates. */
        void ShowJointCoordinate(void) { _bShowJointCoordinate = true; }

        /*!	Hide JointCoordinates. */
        void HideJointCoordinate(void) { _bShowJointCoordinate = false; }



        /*!	Get status of CollisionCoordinate showing. */
        bool IsCollisionCoordinateVisible() const { return _bShowCollisionCoordinate; }

        /*!	Show CollisionCoordinates. */
        void ShowCollisionCoordinate(void) { _bShowCollisionCoordinate = true; }

        /*!	Hide CollisionCoordinates. */
        void HideCollisionCoordinate(void) { _bShowCollisionCoordinate = false; }



        /*!	Get status of SensorCoordinate showing. */
        bool IsSensorCoordinateVisible() const { return _bShowSensorCoordinate; }

        /*!	Show SensorCoordinates. */
        void ShowSensorCoordinate(void) { _bShowSensorCoordinate = true; }

        /*!	Hide SensorCoordinates. */
        void HideSensorCoordinate(void) { _bShowSensorCoordinate = false; }







        /*!	Get status of Help showing. */
        bool IsHelpVisible() const { return _bShowHelp; }

        /*!	Show Help. */
        void ShowHelp(void) { _bShowHelp = true; }

        /*!	Hide Help. */
        void HideHelp(void) { _bShowHelp = false; }



        /*!	Get status of CameraUI showing. */
        bool IsCameraUIVisible() const { return _bShowCameraUI; }

        /*!	Show CameraUI. */
        void ShowCameraUI(void) { _bShowCameraUI = true; }

        /*!	Hide CameraUI. */
        void HideCameraUI(void) { _bShowCameraUI = false; }

        /*! Set user-render callback function */
        void SetUserRenderFunction(void (*pfn)(void*), void* pVoid);

    protected:
        bool _bShowSpace;
        bool _bShowLink;
        bool _bShowJoint;
        bool _bShowCollision;
        bool _bShowSensor;

        bool _bShowLinkCoordinate;
        bool _bShowJointCoordinate;
        bool _bShowCollisionCoordinate;
        bool _bShowSensorCoordinate;

        bool _bShowHelp;

        bool _bShowCameraUI;

        void _UpdateSpace(void);
        void _BuildAllDLs(void);

        void _CompileLinkDLs(srgLink* psrgLink);
        void _CompileJointDLs(srgJoint* psrgJoint);
        void _CompileCollisionDLs(srgCollision* psrgCollision);
        void _CompileSensorDLs(srgSensor* psrgSensor);

        void _UpdateViewLayout(void);

        void _SetProjMatrix3D(void);
        void _SetProjMatrix2D(void);
        float _ProjMat3D[16];
        float _ProjMat2D[16];
        int _ViewportX, _ViewportY;
        int _ViewportW, _ViewportH;

        srgCamera* _pCamera;

        void _RenderSpace(void);
        void _RenderLinks(void);
        void _RenderJoints(void);
        void _RenderCollisions(void);
        void _RenderSensors(void);
        void _RenderCameraUI(void);

        void _DrawViewAxis(void);

        // USER CONTROL FUNCTION
        void	_UserRender();
        void	(*m_pfnUserRender)(void* pVoid);
        //static	void	DoNothing(void* pVoid) {};
        void*	m_pvUserRenderData;

        const srSpace* _pSpace;

        _array<srgLink> _gLinks;
        _array<srgJoint> _gJoints;
        _array<srgCollision> _gCollisions;
        _array<srgSensor> _gSensors;

        srgString			_String;
        //srgCamera _DefaultCamera;
};



#endif // __SIMPLERENDERER__


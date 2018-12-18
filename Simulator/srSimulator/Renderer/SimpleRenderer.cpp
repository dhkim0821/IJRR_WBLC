#include "SimpleRenderer.h"
#include "srDyn/srIRSensor.h"
#include "srDyn/srRangeFinder.h"

#define _SLICE_SIZE_		12
#define _STACK_SIZE_		12

static float _T[16];

//////////////////////////////////////////////////////////////////////////
// srSimpleSenderer
//////////////////////////////////////////////////////////////////////////

srSimpleRenderer::srSimpleRenderer():
    _pSpace(NULL),
    _pCamera(NULL),
    //m_pfnUserRender(srSimpleRenderer::DoNothing),
    m_pfnUserRender(NULL),
    m_pvUserRenderData(NULL)
{

}

srSimpleRenderer::~srSimpleRenderer()
{
    int t;
    t = 1;
}

void srSimpleRenderer::Init(void)
{
    _ViewportX = 0;
    _ViewportY = 0;

    _bShowSpace = true;

    _bShowLink = true;
    _bShowJoint = false;
    _bShowCollision = false;
    _bShowSensor = false;

    _bShowLinkCoordinate = false;
    _bShowJointCoordinate = false;
    _bShowCollisionCoordinate = false;
    _bShowSensorCoordinate = false;

    _bShowHelp = false;

    _bShowCameraUI = false;

    // Font
    _String.ChangFont(srgString::MODE_BITMAP, 5);
}

void srSimpleRenderer::SetTarget(const srSpace *pSpace)
{
    _pSpace = pSpace;
    if (_pSpace)
        _UpdateSpace();
}

void srSimpleRenderer::SetCamera(srgCamera *pCamera)
{
    _pCamera = pCamera;
}

void srSimpleRenderer::_UpdateSpace( void )
{
    if ( _pSpace == NULL) return;
    /*
       int OldLinkNum = m_pSpace->m_Links.get_size();
       int OldJointNum = m_pSpace->m_Joints.get_size();
       int OldCollisionNum = m_pSpace->m_Collisions.get_size();
       int OldSensorNum = m_pSpace->m_Sensors.get_size();

    // get data from srSpace
    if (m_pSpace == NULL)
    {
    for (int i = OldLinkNum - 1; i > -1; --i)
    glDeleteLists(m_sLinkStructs[i].m_iDisplayList, 1);
    for (int i = OldJointNum - 1; i > -1; --i)
    glDeleteLists(m_sJointStructs[i].m_iDisplayList, 1);
    for (int i = OldCollisionNum - 1; i > -1; --i)
    glDeleteLists(m_sCollisionStructs[i].m_iDisplayList, 1);
    for (int i = OldSensorNum - 1; i > -1; --i)
    glDeleteLists(m_sSensorStructs[i].m_iDisplayList, 1);

    m_sLinkStructs.set_size(0);
    m_sJointStructs.set_size(0);
    m_sCollisionStructs.set_size(0);
    m_sSensorStructs.set_size(0);

    return;
    }
    */

    int NewLinkNum = _pSpace->m_Links.get_size();
    int NewJointNum = _pSpace->m_Joints.get_size();
    int NewCollisionNum = _pSpace->m_Collisions.get_size();
    int NewSensorNum = _pSpace->m_Sensors.get_size();

    _gLinks.set_size(NewLinkNum);
    _gJoints.set_size(NewJointNum);
    _gCollisions.set_size(NewCollisionNum);
    _gSensors.set_size(NewSensorNum);

    // Link
    for (int i = NewLinkNum - 1; i > -1; --i)
    {
        _gLinks[i].m_bDLCompiled = false;
        //m_sLinkStructs[i].m_bTextureMatrix = false;
        _gLinks[i].m_iDL_RenderingMode = glGenLists(1);
        _gLinks[i].m_iDL_DesignMode = glGenLists(1);
        _gLinks[i].m_pLink = _pSpace->m_Links[i];
    }

    // Joint
    for (int i = NewJointNum - 1; i > -1; --i)
    {
        _gJoints[i].m_bDLCompiled = false;
        //m_sJointStructs[i].m_bTextureMatrix = false;
        _gJoints[i].m_iDL_RenderingMode = glGenLists(1);
        _gJoints[i].m_iDL_DesignMode = glGenLists(1);
        _gJoints[i].m_pJoint = _pSpace->m_Joints[i];
    }

    // Collision
    for (int i = NewCollisionNum - 1; i > -1; --i)
    {
        _gCollisions[i].m_bDLCompiled = false;
        //m_sCollisionStructs[i].m_bTextureMatrix = false;
        _gCollisions[i].m_iDL_RenderingMode = glGenLists(1);
        _gCollisions[i].m_iDL_DesignMode = glGenLists(1);
        _gCollisions[i].m_pCollision = _pSpace->m_Collisions[i];
    }

    // Sensor
    for (int i = NewSensorNum - 1; i > -1; --i)
    {
        _gSensors[i].m_bDLCompiled = false;
        //m_sSensorStructs[i].m_bTextureMatrix = false;
        _gSensors[i].m_iDL_RenderingMode = glGenLists(1);
        _gSensors[i].m_iDL_DesignMode = glGenLists(1);
        _gSensors[i].m_pSensor = _pSpace->m_Sensors[i];
    }

    _BuildAllDLs();
}

void srSimpleRenderer::_CompileLinkDLs( srgLink* psrgLink )
{
    if ( _pSpace == NULL ) return;

    srGeometryInfo GeomInfo;
    sr_real rDim[3];
    float R, G, B, A;
    srgMaterialColor MColor;

    // Link has only one geometry.
    GeomInfo = psrgLink->m_pLink->GetGeomInfo();
    GeomInfo.GetDimension(rDim[0], rDim[1], rDim[2]);
    GeomInfo.GetColor(R, G, B, A);

    // color setting
    MColor.SetColor(R, G, B, A);

    glNewList(psrgLink->m_iDL_RenderingMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib();
        // MColor.PushAttrib_OnlyColor4();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_FILL, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid_Flexible(m_pCamera->GetHeightIntoSight());
                break;
            case srGeometryInfo::USER:
                break;
            case srGeometryInfo::MESH:
                psrgLink->LoadMeshModel(
                        psrgLink->m_pLink->GetGeomInfo().GetFileName(), 
                        psrgLink->m_pLink->GetGeomInfo().GetMeshScale(),
                        psrgLink->m_pLink->GetGeomInfo().GetLocalFrame());
                break;
            case srGeometryInfo::TDS:
                psrgLink->Load3DSModel(psrgLink->m_pLink->GetGeomInfo().GetFileName(), psrgLink->m_pLink->GetGeomInfo().GetLocalFrame());
                break;
        }
        MColor.PopAttrib();

        glPopAttrib();
    }
    glEndList();

    glNewList(psrgLink->m_iDL_DesignMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib_OnlyColor4();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid_Flexible(m_pCamera->GetHeightIntoSight());
                break;
            case srGeometryInfo::USER:
                break;
            case srGeometryInfo::MESH:
                psrgLink->LoadMeshModel(
                        psrgLink->m_pLink->GetGeomInfo().GetFileName(), 
                        psrgLink->m_pLink->GetGeomInfo().GetMeshScale(),
                        psrgLink->m_pLink->GetGeomInfo().GetLocalFrame());
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::TDS:
                psrgLink->Load3DSModel(psrgLink->m_pLink->GetGeomInfo().GetFileName(), psrgLink->m_pLink->GetGeomInfo().GetLocalFrame());
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
        }
        MColor.PopAttrib_OnlyColor4();

        glPopAttrib();
    }
    glEndList();

    psrgLink->m_bDLCompiled = true;
}

void srSimpleRenderer::_CompileJointDLs( srgJoint* psrgJoint )
{
    if ( _pSpace == NULL ) return;

    srGeometryInfo GeomInfo;
    sr_real rDim[3];
    float R, G, B, A;
    srgMaterialColor MColor;

    // Joint has only one geometry.
    GeomInfo = psrgJoint->m_pJoint->GetGeomInfo();
    GeomInfo.GetDimension(rDim[0], rDim[1], rDim[2]);
    GeomInfo.GetColor(R, G, B, A);

    // color setting
    MColor.SetColor(R, G, B, A);


    glNewList(psrgJoint->m_iDL_RenderingMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_FILL, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid_Flexible(m_pCamera->GetHeightIntoSight());
                break;
            case srGeometryInfo::TDS:
                break;
                            case srGeometryInfo::MESH:
                break;
 
            case srGeometryInfo::USER:
                break;
        }
        MColor.PopAttrib();

        glPopAttrib();
    }
    glEndList();

    glNewList(psrgJoint->m_iDL_DesignMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib_OnlyColor4();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid_Flexible(m_pCamera->GetHeightIntoSight());
                break;
             case srGeometryInfo::MESH:
                break;
           case srGeometryInfo::TDS:
                break;
            case srGeometryInfo::USER:
                break;
        }
        MColor.PushAttrib_OnlyColor4();

        glPopAttrib();
    }
    glEndList();

    psrgJoint->m_bDLCompiled = true;
}

void srSimpleRenderer::_CompileCollisionDLs( srgCollision* psrgCollision )
{
    if ( _pSpace == NULL ) return;

    srGeometryInfo GeomInfo;
    sr_real rDim[3];
    float R, G, B, A;
    srgMaterialColor MColor;

    // Collision has only one geometry.
    GeomInfo = psrgCollision->m_pCollision->GetGeomInfo();
    GeomInfo.GetDimension(rDim[0], rDim[1], rDim[2]);
    GeomInfo.GetColor(R, G, B, A);

    // color setting
    MColor.SetColor(R, G, B, A);


    glNewList(psrgCollision->m_iDL_RenderingMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glDisable(GL_LIGHTING);

        MColor.PushAttrib_OnlyColor4();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid();
                break;
            case srGeometryInfo::TDS:
                break;
             case srGeometryInfo::MESH:
                break;
            case srGeometryInfo::USER:
                break;
        }
        MColor.PopAttrib_OnlyColor4();

        glPopAttrib();
    }
    glEndList();

    glNewList(psrgCollision->m_iDL_DesignMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glDisable(GL_LIGHTING);

        MColor.PushAttrib_OnlyColor4();

        // drawing
        switch ( GeomInfo.GetShape() )
        {
            case srGeometryInfo::SPHERE:
                srgDrawSphere(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::BOX:
                srgDrawBox(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                //_draw_box(rDim);
                break;
            case srGeometryInfo::CAPSULE:
                srgDrawCapsule(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::CYLINDER:
                srgDrawCylinder(SRG_FIGURE_WIREFRAME, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
                glDisable(GL_LIGHTING);
                srgDrawCoordinate_axis_basic(0.2f);
                glEnable(GL_LIGHTING);
                break;
            case srGeometryInfo::PLANE:
                //srgDrawGrid();
                break;
            case srGeometryInfo::TDS:
                break;
            case srGeometryInfo::USER:
                break;
        }
        MColor.PopAttrib_OnlyColor4();

        glPopAttrib();
    }
    glEndList();

    psrgCollision->m_bDLCompiled = true;
}

void srSimpleRenderer::_CompileSensorDLs( srgSensor* psrgSensor )
{
    if ( _pSpace == NULL ) return;

    srGeometryInfo GeomInfo;
    sr_real rDim[3];
    float R, G, B, A;
    srgMaterialColor MColor;

    // Sensor has only one geometry.
    GeomInfo = psrgSensor->m_pSensor->GetGeomInfo();
    GeomInfo.GetDimension(rDim[0], rDim[1], rDim[2]);
    GeomInfo.GetColor(R, G, B, A);

    // color setting
    MColor.SetColor(R, G, B, A);


    glNewList(psrgSensor->m_iDL_RenderingMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib();
        // drawing
        //switch ( GeomInfo.GetShape() )
        //{
        //case srGeometryInfo::SPHERE:
        //	srgDrawSphere(SRG_FIGURE_FILL, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
        //	break;
        //case srGeometryInfo::BOX:
        //	srgDrawBox(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
        //	//_draw_box(rDim);
        //	break;
        //case srGeometryInfo::CAPSULE:
        //	srgDrawCapsule(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
        //	break;
        //case srGeometryInfo::CYLINDER:
        //	srgDrawCylinder(SRG_FIGURE_FILL, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
        //	break;
        //case srGeometryInfo::PLANE:
        //	//srgDrawGrid();
        //	break;
        //case srGeometryInfo::USER:
        //	break;
        //}
        MColor.PopAttrib();

        glPopAttrib();
    }
    glEndList();

    glNewList(psrgSensor->m_iDL_DesignMode, GL_COMPILE);
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);

        MColor.PushAttrib_OnlyColor4();
        // drawing
        //switch ( GeomInfo.GetShape() )
        //{
        //case srGeometryInfo::SPHERE:
        //	srgDrawSphere(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, _SLICE_SIZE_, _STACK_SIZE_);
        //	srgDrawCoordinate_axis_basic(0.2f);
        //	break;
        //case srGeometryInfo::BOX:
        //	srgDrawBox(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1]*0.5f,rDim[2]*0.5f);
        //	srgDrawCoordinate_axis_basic(0.2f);
        //	//_draw_box(rDim);
        //	break;
        //case srGeometryInfo::CAPSULE:
        //	srgDrawCapsule(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
        //	srgDrawCoordinate_axis_basic(0.2f);
        //	break;
        //case srGeometryInfo::CYLINDER:
        //	srgDrawCylinder(SRG_FIGURE_FEATURELINE, rDim[0]*0.5f, rDim[0]*0.5f, rDim[1], _SLICE_SIZE_, _STACK_SIZE_);
        //	srgDrawCoordinate_axis_basic(0.2f);
        //	break;
        //case srGeometryInfo::PLANE:
        //	//srgDrawGrid();
        //	break;
        //case srGeometryInfo::USER:
        //	break;
        //}
        MColor.PopAttrib_OnlyColor4();

        glPopAttrib();
    }
    glEndList();

    psrgSensor->m_bDLCompiled = true;
}

void srSimpleRenderer::_BuildAllDLs(void)
{
    if ( _pSpace == NULL ) return;

    for ( int i = 0; i < _pSpace->m_Links.get_size(); i++ )
        _CompileLinkDLs(&_gLinks[i]);
    for ( int i = 0; i < _pSpace->m_Joints.get_size(); i++ )
        _CompileJointDLs(&_gJoints[i]);
    for ( int i = 0; i < _pSpace->m_Collisions.get_size(); i++ )
        _CompileCollisionDLs(&_gCollisions[i]);
    for ( int i = 0; i < _pSpace->m_Sensors.get_size(); i++ )
        _CompileSensorDLs(&_gSensors[i]);
}

void srSimpleRenderer::RenderSceneOnce(void)
{
    glViewport(_ViewportX, _ViewportY, _ViewportW, _ViewportH);

    // Erase entire screen
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // If there's no space or camera then show notifying messages.
    if ( _pSpace == NULL || _pCamera == NULL )
    {
        glMatrixMode(GL_PROJECTION);
        {
            glLoadMatrixf(_ProjMat2D);
        }

        /* Model-View mode */
        glMatrixMode(GL_MODELVIEW);
        {
            glLoadIdentity();
            //------------------------------------------------------------------------------
            // 1. Background
            //------------------------------------------------------------------------------

            //------------------------------------------------------------------------------
            // 2. Bodies
            //------------------------------------------------------------------------------

            //------------------------------------------------------------------------------
            // 3. UI
            //------------------------------------------------------------------------------
            glPushAttrib(GL_LIGHTING_BIT);
            glPushAttrib(GL_DEPTH_BUFFER_BIT);
            {
                //glDisable(GL_DEPTH_TEST);
                glDisable(GL_LIGHTING);
                srgColor Black(0.0f, 0.0f, 0.0f, 1.0f);
                srgColor Gray(0.5f, 0.5f, 0.5f, 1.0f);
                srgColor White(1.0f, 1.0f, 1.0f, 1.0f);
                srgColor DeepBlue(0.3f, 0.3f, 1.0f);
                srgColor Red(1.0f, 0.0f, 0.0f);

                Red.PushAttrib();
                if ( _pSpace == NULL )
                    _String.Print(5, _ViewportH - 15, "No space");
                if ( _pCamera == NULL )
                    _String.Print(5, _ViewportH - 15, "No camera");
                Red.PopAttrib();

                // Status & LOGO
                Black.PushAttrib();
                _String.Print(5, 20, "[           |         |          |              |            ]");
                _String.Print(5, 5, "SNU Robotics Dynamics Library");
                Black.PushAttrib();

                //glEnable(GL_DEPTH_TEST);
                glEnable(GL_LIGHTING);
            }
            glPopAttrib();
            glPopAttrib();
        }
        return;
    }

    //////////////////////////////////////////////////////////////////////////
    // 3D
    //////////////////////////////////////////////////////////////////////////
    /* Projection mode */
    glMatrixMode(GL_PROJECTION);
    {
        glLoadMatrixf(_ProjMat3D);

    }

    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);
    {
        //------------------------------------------------------------------------------
        // 1. Background
        //------------------------------------------------------------------------------

        //------------------------------------------------------------------------------
        // 2. Bodies
        //------------------------------------------------------------------------------

        // Camera
        if (_pCamera)
            glLoadMatrixd(_pCamera->GetGLCameraFrame());
        else
            glLoadIdentity();

        // Light
        glPushAttrib(GL_LIGHTING_BIT);
        {
            glEnable(GL_LIGHTING);
            //glEnable(GL_COLOR_MATERIAL);
            //glEnable(GL_LIGHT0);
            //glEnable(GL_LIGHT1);
            if (_bShowSpace)
                _RenderSpace();
        }
        glPopAttrib();

        //------------------------------------------------------------------------------
        // 3. User
        //------------------------------------------------------------------------------
        glPushAttrib(GL_LIGHTING_BIT);
        {
            glEnable(GL_LIGHTING);
            _UserRender();
        }
        glPopAttrib();

        //------------------------------------------------------------------------------
        // 4. UI
        //------------------------------------------------------------------------------
        glPushAttrib(GL_LIGHTING_BIT);
        {
            glDisable(GL_LIGHTING);
            _DrawViewAxis();
            //glEnable(GL_LIGHTING);
            if (_bShowCameraUI) _RenderCameraUI();
        }
        glPopAttrib();
    }

    //////////////////////////////////////////////////////////////////////////
    // 2D
    //////////////////////////////////////////////////////////////////////////
    /* Projection mode */
    {
        glMatrixMode(GL_PROJECTION);
        {
            glLoadMatrixf(_ProjMat2D);
        }

        /* Model-View mode */
        glMatrixMode(GL_MODELVIEW);
        {
            glLoadIdentity();
            //------------------------------------------------------------------------------
            // 1. Background
            //------------------------------------------------------------------------------

            //------------------------------------------------------------------------------
            // 2. Bodies
            //------------------------------------------------------------------------------

            //------------------------------------------------------------------------------
            // 3. UI
            //------------------------------------------------------------------------------
            glPushAttrib(GL_LIGHTING_BIT);
            glPushAttrib(GL_DEPTH_BUFFER_BIT);
            {
                //glDisable(GL_DEPTH_TEST);
                glDisable(GL_LIGHTING);
                srgColor Black(0.0f, 0.0f, 0.0f, 1.0f);
                srgColor Gray(0.5f, 0.5f, 0.5f, 1.0f);
                srgColor White(1.0f, 1.0f, 1.0f, 1.0f);
                srgColor DeepBlue(0.3f, 0.3f, 1.0f);

                Black.PushAttrib();
                _String.Print(5, _ViewportH - 15, "Simulation Time: %.3lf", _pSpace->GetSimulationTime());

                _String.Print(5, _ViewportH - 45, "[ Keyboard configuration ]");
                _String.Print(5, _ViewportH - 60, "Num 0: Help show/hide");
                Black.PopAttrib();

                if (_bShowHelp)
                {
                    Black.PushAttrib();

                    // explain your keyboard configuration
                    //_String.Print(5, _ViewportH - 75, "Num 1: Space show/hide");
                    _String.Print(5, _ViewportH - 90, "Num 2: Link show/show with coordinate/hide");
                    _String.Print(5, _ViewportH - 105, "Num 3: Joint show/show with coordinate/hide");
                    _String.Print(5, _ViewportH - 120, "Num 4: Collision show/show with coordinate/hide");
                    _String.Print(5, _ViewportH - 135, "Num 5: Sensor show/show with coordinate/hide");
                    _String.Print(5, _ViewportH - 150, "Space bar: Reset camera");

                    _String.Print(5, _ViewportH - 170, "+/-: Zoom in/out");

                    _String.Print(5, _ViewportH - 190, "Mouse left button: Camera Rotation");
                    _String.Print(5, _ViewportH - 205, "Mouse middle button: Camera Zooming");
                    _String.Print(5, _ViewportH - 220, "Mouse right button: Camera Translation");

                    Black.PushAttrib();
                }

                // Status & LOGO
                if (_bShowSpace)
                {
                    DeepBlue.PushAttrib();
                    _String.Print(5, 20, "  Space                                           ");
                    DeepBlue.PushAttrib();
                }
                else
                {
                    Gray.PushAttrib();
                    _String.Print(5, 20, "  Space                                           ");
                    Gray.PushAttrib();
                }

                if (_bShowLink)
                {
                    Black.PushAttrib();
                    _String.Print(5, 20, "              Link                                  ");
                    if (_bShowLinkCoordinate)
                        _String.Print(5, 20, "              Link'                                  ");
                    Black.PushAttrib();
                }
                else
                {
                    Gray.PushAttrib();
                    _String.Print(5, 20, "              Link                                  ");
                    Gray.PushAttrib();
                }

                if (_bShowJoint)
                {
                    Black.PushAttrib();
                    _String.Print(5, 20, "                         Joint                          ");
                    if (_bShowJointCoordinate)
                        _String.Print(5, 20, "                         Joint'                          ");
                    Black.PushAttrib();
                }
                else
                {
                    Gray.PushAttrib();
                    _String.Print(5, 20, "                         Joint                          ");
                    Gray.PushAttrib();
                }

                if (_bShowCollision)
                {
                    Black.PushAttrib();
                    _String.Print(5, 20, "                                    Collision           ");
                    if (_bShowCollisionCoordinate)
                        _String.Print(5, 20, "                                    Collision'           ");
                    Black.PushAttrib();
                }
                else
                {
                    Gray.PushAttrib();
                    _String.Print(5, 20, "                                    Collision           ");
                    Gray.PushAttrib();
                }

                if (_bShowSensor)
                {
                    Black.PushAttrib();
                    _String.Print(5, 20, "                                                    Sensor  ");
                    if (_bShowSensorCoordinate)
                        _String.Print(5, 20, "                                                    Sensor'  ");
                    Black.PushAttrib();
                }
                else
                {
                    Gray.PushAttrib();
                    _String.Print(5, 20, "                                                    Sensor  ");
                    Gray.PushAttrib();
                }

                Black.PushAttrib();
                _String.Print(5, 20, "[           |          |           |               |             ]");
                _String.Print(5, 5, "SNU Robotics Dynamics Library");
                Black.PushAttrib();

                //glEnable(GL_DEPTH_TEST);
                glEnable(GL_LIGHTING);
            }
            glPopAttrib();
            glPopAttrib();
        }
    }
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void srSimpleRenderer::_DrawViewAxis(void)
{
    // Draw a axis of view in the bottom-left corner.
    // It draws on its region, so don't load matrices of the view.
    //
    // ** Matrix states DOES NOT saved **
    //
    glPushAttrib(GL_VIEWPORT_BIT);
    {
        glPushAttrib(GL_DEPTH_BUFFER_BIT);
        {
            glDisable(GL_DEPTH_TEST);
            // Setup for axis drawing
            glViewport(_ViewportX + _ViewportW - 48, _ViewportY, 48, 48);

            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            {
                glLoadIdentity();
                GLfloat dSize = 32.0f;
                glOrtho(-dSize, dSize, -dSize, dSize, -dSize, dSize);
            }

            // Do global transform to show current rotating states
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            {
                //glLoadIdentity();
                double ViewMat[16];
                _pCamera->m_sCameraInvFrame.ToArray(ViewMat);

                double ViewOrientMatrix[16] = {
                    ViewMat[0], ViewMat[1], ViewMat[2], 0,
                    ViewMat[4], ViewMat[5], ViewMat[6], 0,
                    ViewMat[8], ViewMat[9], ViewMat[10], 0,
                    0, 0, 0, 1
                };
                glLoadMatrixd(ViewOrientMatrix);
                srgDrawCoordinate_axis_basic();
            }
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            glEnable(GL_DEPTH_TEST);
        }
        glPopAttrib();
    }
    glPopAttrib();
    //glMatrixMode(GL_MODELVIEW);
}

void srSimpleRenderer::_RenderSpace(void)
{
    if (_bShowJoint)
        _RenderJoints();
    if (_bShowCollision)
        _RenderCollisions();
    if (_bShowSensor)
        _RenderSensors();
    if (_bShowLink)
        _RenderLinks();
}

void srSimpleRenderer::_RenderLinks( void )
{
    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);

    for ( int i = 0; i < _gLinks.get_size(); i++ )
    {
        _gLinks[i].m_pLink->GetFrame().ToArray(_T);
        glPushMatrix();
        {
            glMultMatrixf(_T);

            // StaticRender
            if (!_gLinks[i].m_bDLCompiled)
                _CompileLinkDLs(&_gLinks[i]);

            if (_bShowLinkCoordinate)
                glCallList(_gLinks[i].m_iDL_DesignMode);
            else
                glCallList(_gLinks[i].m_iDL_RenderingMode);

            // DynmicRender
            srgLink::DYNAMIC_RENDER_DATA DRD;
            DRD.ViewportHeight = _ViewportH;
            DRD.SightHeight = _pCamera->GetHeightOfScene();

            _gLinks[i].DynamicRender(&DRD);
        }
        glPopMatrix();
    }
}

void srSimpleRenderer::_RenderJoints( void )
{
    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);

    for ( int i = 0; i < _gJoints.get_size(); i++ )
    {
        _gJoints[i].m_pJoint->GetFrame().ToArray(_T);
        glPushMatrix();
        {
            glMultMatrixf(_T);
            if (!_gJoints[i].m_bDLCompiled)
                _CompileJointDLs(&_gJoints[i]);

            if (_bShowJointCoordinate)
                glCallList(_gJoints[i].m_iDL_DesignMode);
            else
                glCallList(_gJoints[i].m_iDL_RenderingMode);

            // DynmicRender
            srgJoint::DYNAMIC_RENDER_DATA DRD;
            DRD.ViewportHeight = _ViewportH;
            DRD.SightHeight = _pCamera->GetHeightOfScene();

            _gJoints[i].DynamicRender(&DRD);
        }
        glPopMatrix();
    }
}

void srSimpleRenderer::_RenderCollisions( void )
{
    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);

    for ( int i = 0; i < _gCollisions.get_size(); i++ )
    {
        _gCollisions[i].m_pCollision->GetFrame().ToArray(_T);
        glPushMatrix();
        {
            glDisable(GL_LIGHTING);
            glMultMatrixf(_T);
            if (!_gCollisions[i].m_bDLCompiled)
                _CompileCollisionDLs(&_gCollisions[i]);

            if (_bShowCollisionCoordinate)
                glCallList(_gCollisions[i].m_iDL_DesignMode);
            else
                glCallList(_gCollisions[i].m_iDL_RenderingMode);

            glEnable(GL_LIGHTING);
        }
        glPopMatrix();
    }
}

void srSimpleRenderer::_RenderSensors( void )
{
    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);

    for ( int i = 0; i < _gSensors.get_size(); i++ )
    {
        _gSensors[i].m_pSensor->GetFrame().ToArray(_T);
        glPushMatrix();
        {
            glMultMatrixf(_T);
            if (!_gSensors[i].m_bDLCompiled)
                _CompileSensorDLs(&_gSensors[i]);
            glCallList(_gSensors[i].m_iDL_RenderingMode);

            if(_gSensors[i].m_pSensor->m_SensorType == srSensor::IRSENSOR)
            {
                if (_bShowSensorCoordinate)
                {
                    glDisable(GL_LIGHTING);
                    srgDrawCoordinate_axis_basic(0.2f);
                    glEnable(GL_LIGHTING);
                }

                srIRSensor* sensor = reinterpret_cast<srIRSensor*>(_gSensors[i].m_pSensor);
                sr_real dValue = (sensor->GetDetectedValue() < 0.0) ? 0.0 : sensor->GetDetectedValue();
                srgColor CFColor(0.2f, 1.0f, 0.2f, 0.5f);
                CFColor.PushAttrib();
                glDisable(GL_LIGHTING);
                glBegin(GL_LINES);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(dValue, 0.0, 0.0);
                glEnd();
                glColor3f(1.0, 0.0, 0.0);
                glTranslatef(dValue, 0.0, 0.0);
                srgDrawSphere(SRG_FIGURE_WIREFRAME, 0.0075, _SLICE_SIZE_, _STACK_SIZE_);
                CFColor.PopAttrib();
            }
            else if(_gSensors[i].m_pSensor->m_SensorType == srSensor::RANGEFINDER)
            {
                if (_bShowSensorCoordinate)
                {
                    glDisable(GL_LIGHTING);
                    srgDrawCoordinate_axis_basic(0.2f);
                    glEnable(GL_LIGHTING);
                }

                srRangeFinder* sensor = reinterpret_cast<srRangeFinder*>(_gSensors[i].m_pSensor);
                int num = sensor->GetNumSpots();
                sr_real* res = sensor->GetDetectedValue();
                float fTheta;
                sr_real dis;

                srgColor CFColor(0.2f, 1.0f, 0.2f, 0.5f);
                CFColor.PushAttrib();
                glDisable(GL_LIGHTING);
                glBegin(GL_LINE_LOOP);
                glVertex3f(0.0, 0.0, 0.0);
                while(num--) {
                    dis = (res[num] < 0.0) ? 0.0 : res[num];
                    fTheta = sensor->m_ResRad * num - sensor->m_SpreadRad;
                    glVertex3f(dis*cos(fTheta), dis*sin(fTheta), 0.0);
                }
                glVertex3f(0.0, 0.0, 0.0);
                glEnd();
                CFColor.PopAttrib();
            }
        }
        glPopMatrix();
    }

    // Test code for visualizing spring elements.
    // Jaeyoung Haan, March 24 2009.
    for(int i = 0 ; i < _pSpace->m_Springs.get_size() ; ++i)
    {
        if((_pSpace->m_Springs[i])->IsActive())
        {
            Vec3 l = (_pSpace->m_Springs[i])->GetLeftEnd();
            Vec3 r = (_pSpace->m_Springs[i])->GetRightEnd();
            srgColor CFColor(1.0f, 0.0f, 1.0f, 1.0f);
            CFColor.PushAttrib();
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            glVertex3f(l[0], l[1], l[2]);
            glVertex3f(r[0], r[1], r[2]);
            glEnd();
            CFColor.PopAttrib();
        }
    }

    // Test code for visualizing COM & ZMP of each robot(system)
    // Jaeyoung Haan, April 1 2009.
    for(int i = 0 ; i < _pSpace->m_Systems.get_size() ; ++i)
    {
        Vec3 com = (_pSpace->m_Systems[i])->GetCOM();
        Vec3 zmp = (_pSpace->m_Systems[i])->GetZMP();

        srgColor CFColor(1.0f, 0.0f, 0.0, 1.0f);
        CFColor.PushAttrib();
        glDisable(GL_LIGHTING);
        glPushMatrix();
        glTranslatef(com[0], com[1], com[2]);
        srgDrawSphere(1, 0.03, 16, 8);
        glPopMatrix();
        CFColor.PopAttrib();

        CFColor.SetRGB(0.0f, 0.0f, 1.0f);
        CFColor.PushAttrib();
        glPushMatrix();
        glTranslatef(zmp[0], zmp[1], zmp[2]);
        srgDrawSphere(1, 0.03, 16, 8);
        glPopMatrix();
        CFColor.PopAttrib();
    }
}

void srSimpleRenderer::_RenderCameraUI( void )
{
    /* Model-View mode */
    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();
    {
        // get position of camera focus(center)
        Vec3 Pos = _pCamera->GetCenter();
        // move to the position of camera focus
        glTranslatef(Pos[0], Pos[1], Pos[2]);
        // prepare the red pencil
        //srgMaterialColor CFColor(1.0f, 0.0f, 0.0f, 1.0f);
        srgColor CFColor(1.0f, 0.0f, 0.0f, 1.0f);

        float fHeightOfScene = _pCamera->GetHeightOfScene();
        float fHeightOfWindow = (float)_ViewportH;
        float fScale = fHeightOfScene/fHeightOfWindow*5.0f;

        // with red color
        CFColor.PushAttrib();
        glScalef(fScale, fScale, fScale);
        glDisable(GL_LIGHTING);
        // call draw function
        srgDrawCameraFocus();
        glEnable(GL_LIGHTING);
        glScalef(0.1f, 0.1f, 0.1f);
        // restore to original color
        CFColor.PopAttrib();
    }
    glPopMatrix();
}

void srSimpleRenderer::UpdateWindowSize( int w, int h )
{
    // Fit view-port to window
    _ViewportX = 0;
    _ViewportY = 0;

    _ViewportW = w;
    _ViewportH = h;

    _UpdateViewLayout();
}

void srSimpleRenderer::_UpdateViewLayout( void )
{
    _SetProjMatrix3D();
    _SetProjMatrix2D();
}

void srSimpleRenderer::_SetProjMatrix3D( void )
{
    // Projection matrix setup
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    {
        glLoadIdentity();

        float Aspect = (float)_ViewportW/(float)_ViewportH;

        //if (1)	// Perspective
        //{
        gluPerspective(_pCamera->m_dFOVy, Aspect, _pCamera->GetNear(), _pCamera->GetFar());
        //} 
        //else	// Orthogonal
        //{
        //	double dSize = tan(SRG_DEG2RAD(m_pCamera->GetFOVy()*0.5))*m_pCamera->GetFocusLength();
        //	glOrtho(-dSize*Aspect, dSize*Aspect, -dSize, dSize, m_pCamera->GetNear(), m_pCamera->GetFar());
        //}

        // Save the projection matrix
        glGetFloatv(GL_PROJECTION_MATRIX, _ProjMat3D);
    }
    glPopMatrix();
}

void srSimpleRenderer::_SetProjMatrix2D( void )
{
    // Projection matrix setup
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    {
        glLoadIdentity();

        gluOrtho2D(_ViewportX, _ViewportW, _ViewportY, _ViewportH);

        // Save the projection matrix
        glGetFloatv(GL_PROJECTION_MATRIX, _ProjMat2D);
    }
    glPopMatrix();
}

const int srSimpleRenderer::GetViewportX() const
{
    return _ViewportX;
}

const int srSimpleRenderer::GetViewportY() const
{
    return _ViewportY;
}

const int srSimpleRenderer::GetViewportW() const
{
    return _ViewportW;
}

const int srSimpleRenderer::GetViewportH() const
{
    return _ViewportH;
}

void srSimpleRenderer::SetUserRenderFunction(void (*pfn)(void*), void* pVoid)
{
    m_pfnUserRender = pfn;
    m_pvUserRenderData = pVoid;
}

void srSimpleRenderer::_UserRender()
{
    if (m_pfnUserRender)
        (*m_pfnUserRender)(m_pvUserRenderData);
}

#include "SimpleViewer.h"


//////////////////////////////////////////////////////////////////////////
// srSimpleViewer
//////////////////////////////////////////////////////////////////////////
srSimpleRenderer* srSimpleViewer::s_pRenderer = new srSimpleRenderer;


srgCamera srSimpleViewer::s_Camera = srgCamera();

bool srSimpleViewer::s_bLButton = false;
bool srSimpleViewer::s_bMButton = false;
bool srSimpleViewer::s_bRButton = false;

int srSimpleViewer::s_X = 0;
int srSimpleViewer::s_Y = 0;

int srSimpleViewer::s_X_Buff[] = {0, 0, 0};
int srSimpleViewer::s_Y_Buff[] = {0, 0, 0};

srSpace* srSimpleViewer::s_pSpace = NULL;

// int srSimpleViewer::s_WinSizeX = 500;
// int srSimpleViewer::s_WinSizeY = 500;
//int srSimpleViewer::s_WinSizeX = 1280;
int srSimpleViewer::s_WinSizeX = 1580;
int srSimpleViewer::s_WinSizeY = 720;

_array<srSimpleViewer::KeyFnMap>* srSimpleViewer::s_pKeyFnMaps = new _array<srSimpleViewer::KeyFnMap>();
srSimpleViewer::SRSV_PFNDELEGATE srSimpleViewer::s_pfnLoopFunc = NULL;
srSimpleViewer::SRSV_PFNUSERKEYFUNC srSimpleViewer::s_pfnUserKeyFunc = NULL;
void* srSimpleViewer::s_pvUserKeyData = NULL;

bool MojaveWorkAround = true;


srSimpleViewer::srSimpleViewer(void)
{
}

srSimpleViewer::~srSimpleViewer()
{
	SR_SAFE_DELETE(s_pRenderer);
}

void srSimpleViewer::Init( int *argc, char **argv, const char* title)
{
	glutInit(argc, argv);
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_ALPHA );
	glutInitWindowSize(s_WinSizeX, s_WinSizeY);
	glutCreateWindow(title);

// 	if( glewInit() != GLEW_OK )
// 		printf("Si");

	glutKeyboardFunc(srSimpleViewer::CBKeyboardFunc);
	glutDisplayFunc(srSimpleViewer::CBDisplayFunc);
	glutMouseFunc(srSimpleViewer::CBMouseFunc);
	glutPassiveMotionFunc(srSimpleViewer::CBPassiveMotionFunc);
	glutMotionFunc(srSimpleViewer::CBMotionFunc);
	glutReshapeFunc(srSimpleViewer::CBReshapeFunc);
	glutIdleFunc(srSimpleViewer::CBIdleFunc);

	// Setup RC;
	//glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	//glShadeModel(GL_SMOOTH);

	// Lighting setting
	//GLfloat lightPos[] = {10.0f, 10.0f, -20.0f, 0.0f};
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	// Polygon setting
	glPolygonMode (GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_LINE);
	glEnable(GL_CULL_FACE);
	//glEnable(GL_DEPTH_TEST);
	glCullFace(GL_BACK);

	// Renderer setting
    s_pRenderer->Init();
	s_pRenderer->SetCamera(&s_Camera);

	//s_pRenderer->UpdateWindowSize(500, 500);
     //s_pRenderer->UpdateWindowSize(1280, 820);

	// Camera setting
	s_Camera.Reset();
}

void srSimpleViewer::CBDisplayFunc(void)
{
    if(MojaveWorkAround)
    {
        //Necessary for Mojave. Has to be different dimensions than in 
        // glutInitWindowSize();
        glutReshapeWindow(s_WinSizeX-1, s_WinSizeY-1);
        MojaveWorkAround = false;
    }
    glutPostRedisplay();//Necessary for Mojave.
	s_pRenderer->RenderSceneOnce();
	glutSwapBuffers();
}

void srSimpleViewer::CBKeyboardFunc(unsigned char key, int /*x*/, int /*y*/)
{
	// The x and y callback parameters indicate the mouse location
	// ...in window relative coordinates when the key was pressed.
	switch ( key )
	{
	case '0':
		s_pRenderer->IsHelpVisible() ? s_pRenderer->HideHelp() : s_pRenderer->ShowHelp();
		glutPostRedisplay();
		break;
	//case '1':
	//	s_pRenderer->IsSpaceVisible() ? s_pRenderer->HideSpace() : s_pRenderer->ShowSpace();
	//	glutPostRedisplay();
	//	break;
	case '2':
		if (s_pRenderer->IsLinkVisible())
		{
			if (s_pRenderer->IsLinkCoordinateVisible())
			{
				s_pRenderer->HideLink();
			}
			else
			{
				s_pRenderer->ShowLinkCoordinate();
			}
		}
		else
		{
			s_pRenderer->ShowLink();
			s_pRenderer->HideLinkCoordinate();
		}
		glutPostRedisplay();
		break;
	case '3':
		if (s_pRenderer->IsJointVisible())
		{
			if (s_pRenderer->IsJointCoordinateVisible())
			{
				s_pRenderer->HideJoint();
			}
			else
			{
				s_pRenderer->ShowJointCoordinate();
			}
		}
		else
		{
			s_pRenderer->ShowJoint();
			s_pRenderer->HideJointCoordinate();
		}
		glutPostRedisplay();
		break;
	case '4':
		if (s_pRenderer->IsCollisionVisible())
		{
			if (s_pRenderer->IsCollisionCoordinateVisible())
			{
				s_pRenderer->HideCollision();
			}
			else
			{
				s_pRenderer->ShowCollisionCoordinate();
			}
		}
		else
		{
            s_pRenderer->ShowCollision();
            s_pRenderer->HideCollisionCoordinate();
		}
		glutPostRedisplay();
		break;
	case '5':
		if (s_pRenderer->IsSensorVisible())
		{
			if (s_pRenderer->IsSensorCoordinateVisible())
			{
				s_pRenderer->HideSensor();
			}
			else
			{
				s_pRenderer->ShowSensorCoordinate();
			}
		}
		else
		{
			s_pRenderer->ShowSensor();
			s_pRenderer->HideSensorCoordinate();
		}
		glutPostRedisplay();
		break;

	case '+':
		s_Camera.Approach_Smart(1);
		break;
	case '-':
		s_Camera.Approach_Smart(-1);
		break;

	case 27:	// ESC
		exit(0);
		break;
	case ' ':	// SPACEBAR
		s_Camera.Reset();
		glutPostRedisplay();
		break;
	}

	DoKeyFunc(key);
	DoUserKeyFunc(key);
}

void srSimpleViewer::Run( void )
{
    glutMainLoop();
}

void srSimpleViewer::_Render( void )
{
	s_pRenderer->RenderSceneOnce();
}


void srSimpleViewer::SetTarget( srSpace* pSpace )
{
    s_pRenderer->SetCamera(&s_Camera);
    s_pRenderer->SetTarget(pSpace);
    s_pSpace = pSpace;
}

void srSimpleViewer::CBMouseFunc( int button, int state, int x, int y )
{
	s_X = x;
	s_Y = y;

	s_X_Buff[0] = s_X_Buff[1] = s_X_Buff[2] = x;
	s_Y_Buff[0] = s_Y_Buff[1] = s_Y_Buff[2] = y;

	if (!state)
	{
		s_pRenderer->ShowCameraUI();
	} 
	else
	{
		s_pRenderer->HideCameraUI();
	}

	switch ( button )
	{
	case GLUT_LEFT_BUTTON:
		s_bLButton = !state;
		break;
	case GLUT_MIDDLE_BUTTON:
		s_bMButton = !state;
		break;
	case GLUT_RIGHT_BUTTON:
		s_bRButton = !state;
		break;
	//case GLUT_WHEEL_UP:
	//	s_Camera.Approach_Smart(5);
	//	break;
	//case GLUT_WHEEL_DOWN:
	//	s_Camera.Approach_Smart(-5);
	//	break;
	}
}

void srSimpleViewer::CBMotionFunc( int x, int y )
{
	if (s_bLButton)
	{
		// Translation
		s_Camera.Translate_View(s_X, y, x, s_Y, s_pRenderer->GetViewportH());
	}

	if (s_bMButton)
	{
		// Zoom
		s_Camera.Approach_Smart(s_Y - y);
	}

	if (s_bRButton)
	{
		// Rotate
		s_Camera.Turn_Focus(s_X, y, x, s_Y, 0, 0, s_pRenderer->GetViewportW(), s_pRenderer->GetViewportH(), s_WinSizeY*0.4);
	}

	s_X = x;
	s_Y = y;
}

void srSimpleViewer::CBReshapeFunc( int width, int height )
{
	s_WinSizeX = width;
	s_WinSizeY = height;

	s_pRenderer->UpdateWindowSize(width, height);
}

void srSimpleViewer::CBIdleFunc( void )
{
	DoLoopFunc();
	// step forward of simulation
	//if (s_bSimulationRun)
	//	s_pSpace->DYN_MODE_RUNTIME_SIMULATION_LOOP();

	glutPostRedisplay();
}

void srSimpleViewer::CBPassiveMotionFunc( int /*x*/, int /*y*/ )
{
}

void srSimpleViewer::SetKeyFunc( SRSV_PFNDELEGATE pfn, char Key )
{
	// One function for one key
	for (int i = s_pKeyFnMaps->get_size() - 1; i > -1; --i)
	{
		if (s_pKeyFnMaps->get_at(i).Key == Key)
		{
			s_pKeyFnMaps->get_at(i).pfnKeyFunc = pfn;
			return;
		}
	}
	KeyFnMap NewKeyFnMap;
	NewKeyFnMap.Key = Key;
	NewKeyFnMap.pfnKeyFunc = pfn;
	s_pKeyFnMaps->add_head(NewKeyFnMap);
}

void srSimpleViewer::RemoveKeyFunc( SRSV_PFNDELEGATE pfn )
{
	int i = 0;
	while (i < s_pKeyFnMaps->get_size())
	{
		if (s_pKeyFnMaps->get_at(i).pfnKeyFunc == pfn)
		{
			s_pKeyFnMaps->pop(i);
			i = 0;
		}
		else
		{
			++i;
		}
	}
}

void srSimpleViewer::RemoveKeyFunc( char Key )
{
	int i = 0;
	while (i < s_pKeyFnMaps->get_size())
	{
		if (s_pKeyFnMaps->get_at(i).Key == Key)
		{
			s_pKeyFnMaps->pop(i);
			i = 0;
		}
		else
		{
			++i;
		}
	}
}


void srSimpleViewer::DoKeyFunc( char Key )
{
	if (('a' <= Key && Key <= 'z') || ('A' <= Key && Key <= 'Z'))
	{
		for (int i = s_pKeyFnMaps->get_size() - 1; i > -1; --i)
		{
			if (s_pKeyFnMaps->get_at(i).Key == Key && s_pKeyFnMaps->get_at(i).pfnKeyFunc != NULL)
			{
				(*s_pKeyFnMaps->get_at(i).pfnKeyFunc)();
				glutPostRedisplay();
				return;
			}
		}
	}
}

void srSimpleViewer::DoUserKeyFunc( char Key )
{
	if (s_pfnUserKeyFunc)
		(*s_pfnUserKeyFunc)(Key, s_pvUserKeyData);
}

void srSimpleViewer::SetLoopFunc( SRSV_PFNDELEGATE pfn )
{
	s_pfnLoopFunc = pfn;
}

void srSimpleViewer::RemoveLoopFunc( void )
{
	s_pfnLoopFunc = NULL;
}

void srSimpleViewer::DoLoopFunc( void )
{
	if (s_pfnLoopFunc == NULL) return;

	(*s_pfnLoopFunc)();
}

srSimpleViewer& srSimpleViewer::GetInstance()
{
	static srSimpleViewer SingleSimpleViewer;
	return SingleSimpleViewer;
}

void srSimpleViewer::SetUserRenderFunc( srSimpleRenderer::SRSR_PFNDELEGATE pfn, void* pvData )
{
	s_pRenderer->SetUserRenderFunction(pfn, pvData);
}

void srSimpleViewer::RemoveRenderFunc( void )
{
	s_pRenderer->SetUserRenderFunction(NULL, NULL);
}

void srSimpleViewer::SetUserKeyFunc( SRSV_PFNUSERKEYFUNC pfn, void* pvData )
{
	s_pfnUserKeyFunc = pfn;
	s_pvUserKeyData = pvData;
}


void srgLink::DynamicRender( void* pVoid )
{
	if (pVoid == NULL) return;
	
	DYNAMIC_RENDER_DATA* pDRD = static_cast<DYNAMIC_RENDER_DATA*>(pVoid);

	srGeometryInfo GeomInfo;
	//real rDim[3];
	float R, G, B, A;
	srgMaterialColor MColor;


	// Sensor has only one geometry.
	GeomInfo = m_pLink->GetGeomInfo();
	GeomInfo.GetColor(R, G, B, A);

	// color setting
	MColor.SetColor(R, G, B, A);
	MColor.PushAttrib();

	// drawing
	switch ( GeomInfo.GetShape() )
	{
	case srGeometryInfo::SPHERE:
		break;
	case srGeometryInfo::BOX:
		break;
	case srGeometryInfo::CAPSULE:
		break;
	case srGeometryInfo::CYLINDER:
		break;
	case srGeometryInfo::PLANE:
		glPushAttrib(GL_LIGHTING_BIT);
		glDisable(GL_LIGHTING);
		srgDrawGrid_Flexible(pDRD->ViewportHeight, pDRD->SightHeight);
		glPopAttrib();
		break;
  case srGeometryInfo::TDS:
		break;
	case srGeometryInfo::USER:
		break;
    default:
        break;
	}

	MColor.PopAttrib();
}

void srgJoint::DynamicRender( void* /*pVoid*/ )
{
	//if (pVoid == NULL) return;

	//DYNAMIC_RENDER_DATA* pDRD = static_cast<DYNAMIC_RENDER_DATA*>(pVoid);

	//srGeometryInfo GeomInfo;
	//real rDim[3];
	//float R, G, B, A;
	//srgMaterialColor MColor;

	//// Sensor has only one geometry.
	//GeomInfo = m_pJoint->GetGeomInfo();
	//GeomInfo.GetColor(R, G, B, A);

	//// color setting
	//MColor.SetColor(R, G, B, A);
	//MColor.PushAttrib();

	//float fBoundRadius = pDRD->SightHeight/float(pDRD->ViewportHeight)*10.0;

	//// drawing
	//// enum JOINTTYPE { REVOLUTE, PRISMATIC, UNIVERSAL, WELD };
	//switch ( m_pJoint->m_JointType )
	//{
	//case srJoint::REVOLUTE:
	//	srgDrawRevoluteJoint(SRG_FIGURE_FILL, fBoundRadius, 16, 16);
	//	break;
	//case srJoint::PRISMATIC:
	//	srgDrawPrismaticJoint(SRG_FIGURE_FILL, fBoundRadius, 16, 16);
	//	break;
	//case srJoint::UNIVERSAL:
	//	srgDrawRevoluteJoint(SRG_FIGURE_FILL, fBoundRadius, 16, 16);
	//	break;
	//case srJoint::WELD:
	//	srgDrawWeldJoint(SRG_FIGURE_FILL, fBoundRadius, 16, 16);
	//	break;
	//}

	//MColor.PopAttrib();
}

void srgCollision::DynamicRender( void* /*pVoid*/ )
{

}

void srgSensor::DynamicRender( void* /*pVoid*/ )
{

}

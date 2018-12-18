/*!*****************************************************************************
  [ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-11			Jeongseok Lee
v0.005 : 2009-01-23			Jeongseok Lee
 *******************************************************************************/

#ifndef __SIMPLE_VIEWER__
#define __SIMPLE_VIEWER__

#include "common/utils.h"
#include "srDyn/srSpace.h"
#include "SimpleRenderer.h"

// Needs GLUT
// Windows
#ifdef WIN32
#define GLUT_DISABLE_ATEXIT_HACK
#include <gl/GLut.h>
#endif

// Mac OS X
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

// LINUX
#ifdef linux
#include <GL/glut.h>
#endif

/*!	\brief Renderer for space.
  Simple renderer using OpenGL GLUT.
  Simple viewer is singleton class.
  */
class srSimpleViewer
{
    public:
        /*!	Get instance. */
        static srSimpleViewer& GetInstance();

        /*!	Destructor. */
        ~srSimpleViewer();

        /*!	Initialization. */
        void Init(int *argcp, char **argv, const char* title = "SNU Robotics Dynamics Library");

        /*!	Set target space to render. */
        void SetTarget(srSpace* pSpace);

        /*!	Start glutMainLoop. */
        void Run(void);

        /*! Callback function prototype */
        typedef void(*SRSV_PFNDELEGATE)(void);

        /*! Callback function prototype */
        typedef void(*SRSV_PFNUSERKEYFUNC)(char Key, void* pvData);

        struct KeyFnMap
        {
            char Key;						// key mapped
            SRSV_PFNDELEGATE pfnKeyFunc;	// function pointer
        };

        /*!	Set key function. */
        void SetKeyFunc(SRSV_PFNDELEGATE pfn, char Key);

        /*!	Set user key function. */
        void SetUserKeyFunc(SRSV_PFNUSERKEYFUNC pfn, void* pvData);

        /*!	Remove key function. */
        void RemoveKeyFunc(SRSV_PFNDELEGATE pfn );

        /*!	Remove key function. */
        void RemoveKeyFunc(char Key);

        /*!	Set main loop function. */
        void SetLoopFunc(SRSV_PFNDELEGATE pfn);

        /*!	Remove main loop function. */
        void RemoveLoopFunc(void);

        /*! Set user-render callback function */
        void SetUserRenderFunc(srSimpleRenderer::SRSR_PFNDELEGATE pfn, void* pvData);

        /*! Remove user-render callback function */
        void RemoveRenderFunc(void);

    protected:
        // for singleton
        /*!	Constructor as */
        srSimpleViewer(void);

        static void CBDisplayFunc(void);
        static void CBReshapeFunc(int width, int height);
        static void CBKeyboardFunc(unsigned char key, int x, int y);
        static void CBMouseFunc(int button, int state, int x, int y);
        static void CBMotionFunc(int x, int y);
        static void CBPassiveMotionFunc(int x, int y);
        static void CBIdleFunc(void);

        static _array<KeyFnMap>*			s_pKeyFnMaps;
        static SRSV_PFNDELEGATE				s_pfnLoopFunc;
        static SRSV_PFNUSERKEYFUNC			s_pfnUserKeyFunc;
        static void*						s_pvUserKeyData;
        static void	DoKeyFunc(char Key);
        static void DoUserKeyFunc(char Key);
        static void	DoLoopFunc(void);	

        static srSimpleRenderer*			s_pRenderer;
        static srgCamera					s_Camera;

        static	bool						s_bLButton;
        static	bool						s_bMButton;
        static	bool						s_bRButton;

        static int							s_X;
        static int							s_Y;
        static int							s_X_Buff[3];
        static int							s_Y_Buff[3];

        static srSpace*						s_pSpace;

        static int							s_WinSizeX;
        static int							s_WinSizeY;

        void _Render(void);
};



#endif // __SIMPLE_VIEWER__


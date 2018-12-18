#include "Renderer/SimpleViewer.h"

#include "srg/srgGeometryDraw.h"	// for User rendering
#include "draco_Dyn_environment.h"

#include <vector>
#include <err.h>
#include <signal.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <cstring>

using namespace std;

srSimpleViewer& gViewer = srSimpleViewer::GetInstance();
Draco_Dyn_environment* gDyn_env = new Draco_Dyn_environment();

void User_Simulation_Go_One_Step()
{
  gDyn_env->m_Space->DYN_MODE_RUNTIME_SIMULATION_LOOP(gDyn_env);
}

// When we push the 'O' key, this function will be register to loop
// function of srSimpleViewer. Then the simulation will be stopped.
void User_Simulation_Pause()
{
	// do nothings
}

void User_CBFunc_Run_DYN()
{
	// Run dynamics simulation by setting glMainLoop idle function as dynamics simulation step forward function.
	gViewer.SetLoopFunc(User_Simulation_Go_One_Step);
  // User_Simulation_Go_One_Step();
}

// When we push the 'O' key, this function will work.
void User_CBFunc_Pause_DYN()
{
	// Pause dynamics simulation by setting glMainLoop idle function as empty function.
	gViewer.SetLoopFunc(User_Simulation_Pause);
}

void Initial()
{
	gDyn_env->m_Space->_RestoreInitState();
}

void User_CBFunc_Render(void* pvData)
{
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
  gDyn_env->Rendering_Fnc();
	glEnable(GL_LIGHTING);
	glPopAttrib();
}

void User_SimulationSetting()
{
	gViewer.SetUserRenderFunc(User_CBFunc_Render, NULL);

	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'P');
	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'p');

  gViewer.SetKeyFunc(User_Simulation_Go_One_Step, 'F');
  gViewer.SetKeyFunc(User_Simulation_Go_One_Step, 'f');


	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'O');
	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'o');

	gViewer.SetKeyFunc(Initial,'i');

}
static void usage(int ecode, std::string msg)
{
  errx(ecode,
       "%s\n"
       "  options:\n"
       "  -h               help (this message)\n"
       "  -v               verbose mode\n"
       "  -r  <filename>   robot specification (SAI XML format)\n"
       "  -f  <frequency>  servo rate (integer number in Hz, default 500Hz)\n"
       "  -s  <filename>   skill specification (YAML file with tasks etc)"
       "  -p  <task type>",
       msg.c_str());
}

void parse_options(int argc, char ** argv){
  for (int ii(1); ii< argc; ++ii){
    if((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])){
      usage(EXIT_FAILURE, "problem with option '" + string(argv[ii]) + "'");
    }
    switch(argv[ii][1]){
    case 'o':
      ++ii;
      if( ii >= argc ){
        usage(EXIT_FAILURE, "-o require parameter");
      }
    }
  } 
}

int main(int argc, char **argv) 
{
  parse_options(argc, argv);
  gViewer.Init(&argc, argv, "Dynamic Simulation of Draco");

  gViewer.SetTarget(gDyn_env->m_Space);

  //readd();

  User_SimulationSetting();
  gViewer.Run();
    
  return 0;
}

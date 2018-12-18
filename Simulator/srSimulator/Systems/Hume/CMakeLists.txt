add_executable (simulation_hume
  simulation_hume.cpp 
  Hume.h Hume.cpp 
  Dyn_environment.h Dyn_environment.cpp
  Ground.h
  simulation_setting.h simulation_setting.cpp
  Terrain.h Terrain.cpp
  LED_Position_Announcer.h LED_Position_Announcer.cpp
  )

target_link_libraries (simulation_hume
  common
  LieGroup
  srDyn
  srg
  Renderer
  SJutils
  Hume_Controller
  SJPlanner
  )

target_link_libraries(simulation_hume ${OPENGL_LIBRARIES})
target_link_libraries(simulation_hume pthread)
target_link_libraries(simulation_hume Hume_Controller)                                      

if (APPLE)
  target_link_libraries(simulation_hume "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)

  target_link_libraries(simulation_hume ${GLUT_LIBRARIES})
  include (CheckIncludeFileCXX)
  
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

FILE(GLOB_RECURSE header *.hpp)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_sagitP3 ${header} ${source} )

target_link_libraries (simulation_sagitP3
  common
  LieGroup
  srDyn
  SR_SysGenerator
  srg
  Renderer
  util
  ${OPENGL_LIBRARIES}
  dynacore_urdf
  dynacore_param_handler
  SagitP3_Controller
  )

if (APPLE)
    target_link_libraries(simulation_sagitP3 "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_sagitP3 ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

FILE(GLOB_RECURSE header *.hpp)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_Cassie ${header} ${source} )

target_link_libraries (simulation_Cassie
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
)

if (APPLE)
    target_link_libraries(simulation_Cassie "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
    target_link_libraries(simulation_Cassie ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

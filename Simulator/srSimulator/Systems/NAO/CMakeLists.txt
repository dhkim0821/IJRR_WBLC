FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_nao
${header} ${source}
  )

target_link_libraries (simulation_nao
  common
  LieGroup
  srDyn
  srg
  Renderer
  util
  SR_SysGenerator
  NAO_Controller
)

target_link_libraries(simulation_nao ${OPENGL_LIBRARIES})
target_link_libraries(simulation_nao pthread)

if (APPLE)
  target_link_libraries(simulation_nao "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)

  target_link_libraries(simulation_nao ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)

endif (CMAKE_SYSTEM_NAME MATCHES Linux)

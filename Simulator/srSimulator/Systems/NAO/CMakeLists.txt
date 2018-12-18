FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_NAO
${header} ${source}
  )

target_link_libraries (simulation_NAO
  common
  LieGroup
  srDyn
  srg
  Renderer
  util
  SR_SysGenerator
  #NAO_Controller
  #SR_Terrain
)

target_link_libraries(simulation_NAO ${OPENGL_LIBRARIES})
target_link_libraries(simulation_NAO pthread)

if (APPLE)
  target_link_libraries(simulation_NAO "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)

  target_link_libraries(simulation_NAO ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)

endif (CMAKE_SYSTEM_NAME MATCHES Linux)

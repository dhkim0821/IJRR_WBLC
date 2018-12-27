SET (ModelPath ${PROJECT_SOURCE_DIR}/Simulator/SimulationModel/)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

if (APPLE)
    include_directories("/opt/X11/include/GL/")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(GLUT REQUIRED)
    cmake_policy(SET CMP0003 NEW)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Simulator/srSimulator/srconfig.h.cmake ${PROJECT_SOURCE_DIR}/Simulator/srSimulator/srConfiguration.h)


include_directories ("${PROJECT_SOURCE_DIR}/Simulator/srSimulator")
include_directories ("${PROJECT_SOURCE_DIR}/Simulator/srSimulator/assimp/include/")

add_subdirectory(assimp)
add_subdirectory(Renderer)
add_subdirectory(srDyn)
add_subdirectory(srg)
add_subdirectory(common)
add_subdirectory(LieGroup)
add_subdirectory(libpng)
add_subdirectory(srSysGenerator)

add_subdirectory(Systems/Mercury)
add_subdirectory(Systems/Atlas)
add_subdirectory(Systems/Valkyrie)
add_subdirectory(Systems/DracoBip)

project (Supervisor)
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

set (${PROJECT_NAME}_VERSION_NAME "star")
set (${PROJECT_NAME}_VERSION_MAJOR 1)
set (${PROJECT_NAME}_VERSION_MINOR 0)
set (${PROJECT_NAME}_VERSION_PATCH 0)
set (${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})
set (CMAKE_BUILD_TYPE Debug)
message ("")
message ("-[ MAKE INFORMATION ]-----------------------------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------------------------")
message ("")

include_directories ("${PROJECT_SOURCE_DIR}/../../utils")
include_directories ("${PROJECT_SOURCE_DIR}/../../ExternalSource")
include_directories ("${PROJECT_SOURCE_DIR}/../../")

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})


set (headers 
DataSave.h
data_protocol.h
../../Utils/dynacore_pThread.hpp
../../Utils/wrap_eigen.hpp
../../Utils/comm_udp.hpp
../../Utils/utilities.hpp
)

set (sources
DataSave.cpp
Status_Display.cpp
../../Utils/dynacore_pThread.cpp
../../Utils/wrap_eigen.cpp
../../Utils/comm_udp.cpp
../../Utils/utilities.cpp
)

link_directories("/usr/local/lib/")
link_directories("/usr/lib/")

add_executable(Status_Display ${sources} ${headers})
# add_dependencies(Status_Display comm_comm)
target_link_libraries(Status_Display pthread)

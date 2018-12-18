include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")

if (WIN32)
  add_library(dynacore_utils SHARED
    wrap_eigen.hpp wrap_eigen.cpp
    BSplineBasic.h
    LQR.hpp LQR.cpp
    pseudo_inverse.hpp pseudo_inverse.cpp
    utilities.hpp utilities.cpp
    minjerk_one_dim.hpp
    minjerk_one_dim.cpp)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_utils SHARED ${sources} ${headers} ${cppheaders})

find_package(Threads REQUIRED)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(dynacore_utils PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(dynacore_utils "${CMAKE_THREAD_LIBS_INIT}")
endif()
endif(UNIX)

install(TARGETS dynacore_utils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils")

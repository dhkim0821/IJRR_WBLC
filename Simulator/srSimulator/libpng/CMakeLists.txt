# needed packages

if(NOT WIN32)
  find_library(M_LIBRARY
    NAMES m
    PATHS /usr/lib /usr/local/lib
  )
  if(NOT M_LIBRARY)
    message(STATUS "math lib 'libm' not found; floating point support disabled")
  endif()
else()
  # not needed on windows
  set(M_LIBRARY "")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/pnglibconf.h.prebuilt  
    ${CMAKE_CURRENT_BINARY_DIR}/pnglibconf.h)

# OUR SOURCES
set(libpng_public_hdrs
  png.h
  pngconf.h
  "${CMAKE_CURRENT_BINARY_DIR}/pnglibconf.h"
)
set(libpng_private_hdrs
  pngpriv.h
  pngdebug.h
  pnginfo.h
  pngstruct.h
)
set(libpng_sources
  ${libpng_public_hdrs}
  ${libpng_private_hdrs}
  png.c
  pngerror.c
  pngget.c
  pngmem.c
  pngpread.c
  pngread.c
  pngrio.c
  pngrtran.c
  pngrutil.c
  pngset.c
  pngtrans.c
  pngwio.c
  pngwrite.c
  pngwtran.c
  pngwutil.c
)
add_library(png SHARED ${libpng_sources})
target_link_libraries(png ${M_LIBRARY})

find_package(ZLIB)
IF(ZLIB_FOUND )
    find_package(ZLIB REQUIRED)
    include_directories(${ZLIB_INCLUDE_DIR})
    target_link_libraries(png ${ZLIB_LIBRARY})
else( ZLIB_FOUND)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../assimp/contrib/zlib/)
    target_link_libraries(png zlib)
ENDIF(ZLIB_FOUND)



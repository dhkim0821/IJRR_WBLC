set (headers 
srg.h
srgBitmap.h
srgCamera.h
srgColor.h
srgFont.h
srgGeometryDraw.h
srgL.h
srgLieGroup.h
srgMath.h
)

set (sources
srg.cpp
srgBitmap.cpp
srgCamera.cpp
srgColor.cpp
srgFont.cpp
srgGeometryDraw.cpp
)

add_library(srg ${sources} ${headers})
target_link_libraries(srg common)
target_link_libraries(srg LieGroup)
target_link_libraries(srg ${OPENGL_LIBRARIES})
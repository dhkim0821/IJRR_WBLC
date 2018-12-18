FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)

add_library(SR_SysGenerator ${header} ${source})

target_link_libraries (SR_SysGenerator LieGroup common dynacore_urdf)

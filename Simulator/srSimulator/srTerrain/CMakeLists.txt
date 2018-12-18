FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)

add_library(SR_Terrain ${header} ${source})

target_link_libraries (SR_Terrain  srDyn)

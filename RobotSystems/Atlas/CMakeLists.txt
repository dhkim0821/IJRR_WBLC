FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headerspp *.hpp)
add_library(Atlas_model SHARED ${sources} ${headerspp})

target_link_libraries (Atlas_model dynacore_utils dynacore_rbdl)

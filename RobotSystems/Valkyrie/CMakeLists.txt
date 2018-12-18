FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headerspp *.hpp)
add_library(Valkyrie_model SHARED ${sources} ${headerspp})

target_link_libraries (Valkyrie_model dynacore_utils dynacore_rbdl)

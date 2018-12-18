FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBLC SHARED ${sources} ${headers})
target_link_libraries(WBLC dynacore_utils dynacore_Goldfarb)

install(TARGETS WBLC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/WBLC")

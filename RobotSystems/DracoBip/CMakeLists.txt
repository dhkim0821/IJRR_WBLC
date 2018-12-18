FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE cheaders *.h)

set(CTRL_SYS DracoBip)

add_library(${CTRL_SYS}_model SHARED ${sources} ${headers})

target_link_libraries (${CTRL_SYS}_model dynacore_utils dynacore_rbdl)

install(TARGETS ${CTRL_SYS}_model DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} ${cheaders} DESTINATION
    "${INSTALL_INCLUDE_DIR}/${CTRL_SYS}")

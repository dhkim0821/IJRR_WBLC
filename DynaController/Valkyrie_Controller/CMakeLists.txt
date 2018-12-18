FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

set(CTRL_SYS Valkyrie)

add_library (${CTRL_SYS}_Controller SHARED ${headers} ${sources} ../Test.cpp)
target_link_libraries (${CTRL_SYS}_Controller dynacore_utils dynacore_filters)
target_link_libraries (${CTRL_SYS}_Controller ${CTRL_SYS}_model)
target_link_libraries (${CTRL_SYS}_Controller WBLC)

target_link_libraries (${CTRL_SYS}_Controller dynacore_param_handler PIPM_planner)

install(TARGETS ${CTRL_SYS}_Controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/${CTRL_SYS}_Controller")

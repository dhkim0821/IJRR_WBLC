FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library(PIPM_planner SHARED ${sources} ${headers} ${cppheaders})

target_link_libraries(PIPM_planner dynacore_utils dynacore_param_handler)

install(TARGETS PIPM_planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/Planner/PIPM_FootPlacementPlanner")

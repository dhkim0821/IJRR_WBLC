FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)
add_library (dynacore_Goldfarb SHARED ${headers} ${sources})

install(TARGETS dynacore_Goldfarb DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Optimizer/Goldfarb")

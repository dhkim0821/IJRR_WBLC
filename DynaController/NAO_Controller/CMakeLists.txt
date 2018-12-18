FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

set(CTRL_SYS NAO_Exam)

add_library (${CTRL_SYS}_Controller SHARED ${headers} ${sources} ../Test.cpp)
target_link_libraries (${CTRL_SYS}_Controller dynacore_utils)
target_link_libraries (${CTRL_SYS}_Controller NAO_model)
#target_link_libraries (${CTRL_SYS}_Controller WBDC)

#install(TARGETS ${CTRL_SYS}_Controller DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/DynController/${CTRL_SYS}_Controller")

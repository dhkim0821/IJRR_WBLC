FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cc)

SET ( RBDL_VERSION_MAJOR 2 )
SET ( RBDL_VERSION_MINOR 4 )
SET ( RBDL_VERSION_PATCH 0 )

IF (RBDL_STORE_VERSION)
	# Set versioning information that can be queried during runtime
	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "id -i"
		OUTPUT_VARIABLE RBDL_BUILD_REVISION)
	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "branch"
		OUTPUT_VARIABLE RBDL_BUILD_BRANCH)
	SET (RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE (RBDL_STORE_VERSION)
	SET (RBDL_BUILD_REVISION "unknown")
	SET (RBDL_BUILD_BRANCH "unknown")
	SET (RBDL_BUILD_TYPE "unknown")
ENDIF (RBDL_STORE_VERSION)

CONFIGURE_FILE( ${PROJECT_SOURCE_DIR}/ExternalSource/rbdl/rbdl_config.h.cmake 
    ${PROJECT_SOURCE_DIR}/ExternalSource/rbdl/rbdl_config.h )

add_library(dynacore_rbdl SHARED ${sources} ${headers})
target_link_libraries(dynacore_rbdl dynacore_urdf)

install(TARGETS dynacore_rbdl DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/rbdl")

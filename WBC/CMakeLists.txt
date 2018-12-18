add_subdirectory(WBLC)

install(FILES WBC.hpp Task.hpp ContactSpec.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")

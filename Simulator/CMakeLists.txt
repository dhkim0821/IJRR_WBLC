#add_subdirectory(DART)
add_subdirectory(srSimulator)

SET (SIM_MODEL_PATH "${PROJECT_SOURCE_DIR}/Simulator/SimulationModel" )
SET (SIM_DATA_PATH "${PROJECT_SOURCE_DIR}/Simulator/ResultData")

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Simulator/SIM_config.h.cmake 
    ${PROJECT_SOURCE_DIR}/Simulator/SIM_Configuration.h)



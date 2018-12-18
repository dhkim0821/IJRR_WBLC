set (headers 
srBallJoint.h
srClosedLoopConstraint.h
srCollision.h
srConstraint.h
srContactConstraint.h
srDYN.h
srEntity.h
srGeometryInfo.h
srIRSensor.h
srISLAND.h
srJoint.h
srJointConstraint.h
srLinearSpring.h
srLink.h
srObject.h
srPrismaticJoint.h
srRangeFinder.h
srRevoluteJoint.h
srSensor.h
srSpace.h
srSpring.h
srState.h
srSystem.h
srTouchSensor.h
srUniversalJoint.h
srWeldJoint.h
)

set (sources
srBallJoint.cpp
srClosedLoopConstraint.cpp
srCollision.cpp
srCollisionPair.cpp
srConstraint.cpp
srContactConstraint.cpp
srDYN.cpp
srEntity.cpp
srGeometryInfo.cpp
srIRSensor.cpp
srISLAND.cpp
srJoint.cpp
srJointConstraint.cpp
srLinearSpring.cpp
srLink.cpp
srObject.cpp
srPrismaticJoint.cpp
srRangeFinder.cpp
srRevoluteJoint.cpp
srSensor.cpp
srSpace.cpp
srSpring.cpp
srState.cpp
srSystem.cpp
srTouchSensor.cpp
srUniversalJoint.cpp
srWeldJoint.cpp
)

add_library(srDyn ${sources} ${headers})
target_link_libraries (srDyn common)
target_link_libraries (srDyn LieGroup)

if(RECORD_SIMULATION)
    target_link_libraries (srDyn opengl_recorder)
endif()

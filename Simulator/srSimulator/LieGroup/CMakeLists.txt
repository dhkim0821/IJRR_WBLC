set (headers 
LieGroup.h
LieGroup.inl
)

set (sources
LieGroup.cpp
)

add_library(LieGroup ${sources} ${headers})
target_link_libraries (LieGroup common)
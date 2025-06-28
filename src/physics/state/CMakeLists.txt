# Physics State library
add_library(iloss_physics_state
    StateVector.cpp
    StateHistory.cpp
)

target_include_directories(iloss_physics_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(iloss_physics_state PUBLIC
    iloss_math
    iloss_time
    iloss_coordinates
)

# Set properties
set_target_properties(iloss_physics_state PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Install rules
install(TARGETS iloss_physics_state
    EXPORT iloss_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/physics/state
    DESTINATION include/physics
    FILES_MATCHING PATTERN "*.h"
)
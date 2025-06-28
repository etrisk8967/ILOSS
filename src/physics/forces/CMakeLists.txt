# Force model implementation files
set(FORCE_SOURCES
    ForceAggregator.cpp
    ForceModelRegistry.cpp
    SimpleGravityModel.cpp
    twobody/TwoBodyForceModel.cpp
    twobody/KeplerPropagator.cpp
    twobody/ConicSectionUtilities.cpp
    twobody/TwoBodyAnalyticalPropagator.cpp
)

# Create forces library
add_library(iloss_forces ${FORCE_SOURCES})

# Set include directories
target_include_directories(iloss_forces
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

# Link dependencies
target_link_libraries(iloss_forces
    PUBLIC
        iloss_physics_state
        iloss_math
        iloss_time
        iloss_logging
        iloss_coordinates
        iloss_constants
)

# Set properties
set_target_properties(iloss_forces PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
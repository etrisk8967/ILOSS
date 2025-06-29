# Add subdirectory for gravity models
add_subdirectory(gravity)

# Add subdirectory for two-body dynamics
add_subdirectory(twobody)

# Add subdirectory for third-body perturbations
add_subdirectory(thirdbody)

# Add subdirectory for drag forces
add_subdirectory(drag)

# Add subdirectory for solar radiation pressure
add_subdirectory(srp)

# Force model implementation files
set(FORCE_SOURCES
    ForceAggregator.cpp
    ForceModelRegistry.cpp
    RegisterStandardModels.cpp
    SimpleGravityModel.cpp
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
        iloss_earth_gravity
        iloss_twobody
        iloss_thirdbody
        iloss_drag_force
        iloss_srp
)

# Create alias for force model base
add_library(iloss_force_model ALIAS iloss_forces)

# Set properties
set_target_properties(iloss_forces PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
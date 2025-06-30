# Dynamics module sources
set(DYNAMICS_SOURCES
    DynamicsEngine.cpp
    DynamicsState.cpp
    SimpleMassProperties.cpp
    TorqueAggregator.cpp
    torques/GravityGradientTorque.cpp
)

# Create dynamics library
add_library(iloss_dynamics ${DYNAMICS_SOURCES})

# Set include directories
target_include_directories(iloss_dynamics 
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/include
)

# Link dependencies
target_link_libraries(iloss_dynamics
    PUBLIC
        iloss_math
        iloss_physics_state
        iloss_forces
        iloss_coordinates
    PRIVATE
        iloss_logging
)

# Set compile features
target_compile_features(iloss_dynamics PUBLIC cxx_std_20)

# Add subdirectory for torque models
add_subdirectory(torques)
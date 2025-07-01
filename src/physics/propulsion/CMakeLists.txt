# Propulsion module CMakeLists.txt

# Find all propulsion source files
set(PROPULSION_SOURCES
    PropellantTank.cpp
    Engine.cpp
    ThrustVectorControl.cpp
    PropulsionSystem.cpp
    ThrustForceModel.cpp
)

# Create propulsion library
add_library(iloss_physics_propulsion STATIC ${PROPULSION_SOURCES})

# Set include directories
target_include_directories(iloss_physics_propulsion
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

# Link dependencies
target_link_libraries(iloss_physics_propulsion
    PUBLIC
        iloss_dynamics
        iloss_forces
        iloss_physics_state
        iloss_time
        iloss_constants
        iloss_coordinates
        iloss_logging
)

# Set compile features
target_compile_features(iloss_physics_propulsion PUBLIC cxx_std_20)

# Add compile options
target_compile_options(iloss_physics_propulsion PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Install headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/physics/forces/propulsion
    DESTINATION include/physics/forces
    FILES_MATCHING PATTERN "*.h"
)

# Install library
install(TARGETS iloss_physics_propulsion
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
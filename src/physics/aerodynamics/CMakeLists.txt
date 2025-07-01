# Aerodynamics module source files
set(AERODYNAMICS_SOURCES
    AerodynamicCoefficients.cpp
    AerodynamicDatabase.cpp
    AerodynamicCalculator.cpp
    AerodynamicForceModel.cpp
    AerodynamicTorqueModel.cpp
)

# Create aerodynamics library
add_library(iloss_aerodynamics ${AERODYNAMICS_SOURCES})

# Set include directories
target_include_directories(iloss_aerodynamics
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

# Link dependencies
target_link_libraries(iloss_aerodynamics
    PUBLIC
        iloss_physics_state
        iloss_physics_dynamics
        iloss_forces
        iloss_constants
        iloss_math
        iloss_time
        iloss_logging
)

# Set properties
set_target_properties(iloss_aerodynamics PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Install headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/physics/aerodynamics
    DESTINATION include/physics
    FILES_MATCHING PATTERN "*.h"
)

# Install library
install(TARGETS iloss_aerodynamics
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
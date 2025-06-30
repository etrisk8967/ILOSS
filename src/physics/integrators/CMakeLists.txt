# Integrators library
add_library(iloss_integrators STATIC
    RK4Integrator.cpp
    RK78Integrator.cpp
    StepSizeController.cpp
)

# Set include directories
target_include_directories(iloss_integrators
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(iloss_integrators
    PUBLIC
        iloss_physics_state
        iloss_forces
        iloss_logging
    PRIVATE
        Eigen3::Eigen
)

# Compile features
target_compile_features(iloss_integrators PUBLIC cxx_std_20)

# Install rules
install(TARGETS iloss_integrators
    EXPORT ILOSSTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/physics/integrators
    DESTINATION include/physics
    FILES_MATCHING PATTERN "*.h"
)
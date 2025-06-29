# Solar Radiation Pressure Force Model

# SRP library
add_library(iloss_srp STATIC
    SolarRadiationPressureModel.cpp
)

target_include_directories(iloss_srp PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_srp PUBLIC
    iloss_physics_state
    iloss_math
    iloss_time
    iloss_constants
    iloss_coordinates
    iloss_logging
)

# Set properties
set_target_properties(iloss_srp PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Add to parent targets
if(TARGET iloss_physics_forces)
    target_link_libraries(iloss_physics_forces PUBLIC iloss_srp)
endif()
# Earth Gravity Model library
add_library(iloss_earth_gravity STATIC
    EarthGravityModel.cpp
)

target_include_directories(iloss_earth_gravity PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_earth_gravity PUBLIC
    iloss_physics_state
    iloss_math
    iloss_time
    iloss_constants
    iloss_logging
    iloss_coordinates
)

# Set properties
set_target_properties(iloss_earth_gravity PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
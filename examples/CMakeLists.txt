# Example programs demonstrating ILOSS functionality

# Two-body dynamics example
add_executable(two_body_example two_body_example.cpp)

target_link_libraries(two_body_example
    PRIVATE
        iloss_forces
        iloss_physics_state
        iloss_math
        iloss_time
        iloss_constants
        iloss_coordinates
        iloss_logging
)

set_target_properties(two_body_example PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
)

# Add plugin examples subdirectory
add_subdirectory(plugins)
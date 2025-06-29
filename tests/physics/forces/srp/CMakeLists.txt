# Solar Radiation Pressure Tests

# Create test executable
add_executable(test_srp
    test_SolarRadiationPressure.cpp
)

# Link test dependencies
target_link_libraries(test_srp
    PRIVATE
        gtest
        gtest_main
        iloss_srp
        iloss_physics_state
        iloss_time
        iloss_math
        iloss_constants
        iloss_coordinates
        iloss_logging
)

# Add test to CTest
add_test(NAME test_srp COMMAND test_srp)

# Set test properties
set_tests_properties(test_srp PROPERTIES
    TIMEOUT 120
    LABELS "physics;forces;srp"
)
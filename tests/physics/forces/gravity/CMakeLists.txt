# Earth Gravity Model tests
add_executable(test_EarthGravityModel
    test_EarthGravityModel.cpp
)

target_link_libraries(test_EarthGravityModel
    PRIVATE
        iloss_earth_gravity
        iloss_force_model
        iloss_physics_state
        iloss_time
        iloss_constants
        iloss_math
        GTest::gtest
        GTest::gtest_main
)

# Add test
add_test(NAME test_EarthGravityModel COMMAND test_EarthGravityModel)

# Set test properties
set_tests_properties(test_EarthGravityModel PROPERTIES
    TIMEOUT 30
    LABELS "physics;forces;gravity"
)
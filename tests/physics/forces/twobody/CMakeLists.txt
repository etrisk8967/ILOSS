# Two-body dynamics test files
set(TWOBODY_TEST_SOURCES
    TwoBodyForceModelTest.cpp
    KeplerPropagatorTest.cpp
    ConicSectionUtilitiesTest.cpp
    TwoBodyAnalyticalPropagatorTest.cpp
)

# Create test executable for two-body dynamics
add_executable(test_twobody ${TWOBODY_TEST_SOURCES})

# Link test dependencies
target_link_libraries(test_twobody
    PRIVATE
        iloss_forces
        iloss_physics_state
        iloss_math
        iloss_time
        iloss_constants
        iloss_coordinates
        GTest::gtest
        GTest::gtest_main
)

# Add tests to CTest
add_test(NAME TwoBodyDynamics COMMAND test_twobody)

# Set test properties
set_tests_properties(TwoBodyDynamics PROPERTIES
    LABELS "Physics;Forces;TwoBody"
    TIMEOUT 60
)
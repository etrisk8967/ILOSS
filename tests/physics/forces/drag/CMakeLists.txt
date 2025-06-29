# Drag force model tests
set(DRAG_TEST_SOURCES
    test_DragForceModel.cpp
    test_NRLMSISE00Atmosphere.cpp
)

# Create test executable for drag force tests
add_executable(test_drag_force ${DRAG_TEST_SOURCES})

# Link test dependencies
target_link_libraries(test_drag_force
    PRIVATE
        iloss_drag_force
        iloss_forces
        iloss_physics_state
        iloss_time
        iloss_math
        iloss_constants
        iloss_coordinates
        iloss_logging
        GTest::gtest
        GTest::gtest_main
)

# Add tests to CTest
add_test(NAME test_drag_force_model COMMAND test_drag_force --gtest_filter=DragForceModelTest.*)
add_test(NAME test_nrlmsise00_atmosphere COMMAND test_drag_force --gtest_filter=NRLMSISE00AtmosphereTest.*)
add_test(NAME test_drag_nrlmsise00_integration COMMAND test_drag_force --gtest_filter=DragForceModelNRLMSISE00Test.*)
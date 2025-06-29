# Third-body force model tests
add_executable(test_thirdbody_force_model
    ThirdBodyForceModelTest.cpp
)

target_link_libraries(test_thirdbody_force_model
    PRIVATE
        iloss_thirdbody
        iloss_physics_state
        iloss_time
        iloss_math
        iloss_logging
        GTest::gtest
        GTest::gtest_main
)

# Link SPICE if available
if(SPICE_FOUND)
    # SPICE needs to be linked AFTER iloss_thirdbody since static libraries have order dependencies
    # Also add math library which SPICE might need
    target_link_libraries(test_thirdbody_force_model PRIVATE ${SPICE_LIBRARIES} m)
    target_include_directories(test_thirdbody_force_model PRIVATE ${SPICE_INCLUDE_DIR})
    target_compile_definitions(test_thirdbody_force_model PRIVATE SPICE_FOUND)
endif()

# Add test to CTest
add_test(NAME test_thirdbody_force_model
    COMMAND test_thirdbody_force_model
)

# Set test properties
set_tests_properties(test_thirdbody_force_model PROPERTIES
    TIMEOUT 60
    LABELS "physics;forces;thirdbody"
)
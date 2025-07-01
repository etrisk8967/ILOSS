# Integrator tests
add_executable(test_integrators
    test_RK4Integrator.cpp
    test_RK78Integrator.cpp
    test_StepSizeController.cpp
)

target_link_libraries(test_integrators
    PRIVATE
        iloss_integrators
        iloss_physics_state
        iloss_forces
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

# Register the test
add_test(
    NAME test_integrators
    COMMAND test_integrators
)

# Set test properties
set_tests_properties(test_integrators PROPERTIES
    TIMEOUT 30
    LABELS "physics;integrators"
)

# Coupled dynamics integration test
add_executable(test_CoupledDynamicsIntegration
    test_CoupledDynamicsIntegration.cpp
)

target_link_libraries(test_CoupledDynamicsIntegration
    PRIVATE
        iloss_integrators
        iloss_dynamics
        iloss_physics_state
        iloss_forces
        iloss_constants
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

# Register the test
add_test(
    NAME test_CoupledDynamicsIntegration
    COMMAND test_CoupledDynamicsIntegration
)

# Set test properties
set_tests_properties(test_CoupledDynamicsIntegration PROPERTIES
    TIMEOUT 300
    LABELS "physics;integrators;dynamics"
)
# Dynamics module tests

# Create test executables
add_executable(test_SimpleMassProperties test_SimpleMassProperties.cpp)
target_link_libraries(test_SimpleMassProperties 
    iloss_dynamics
    iloss_math
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME SimpleMassPropertiesTest COMMAND test_SimpleMassProperties)

add_executable(test_DynamicsState test_DynamicsState.cpp)
target_link_libraries(test_DynamicsState 
    iloss_dynamics
    iloss_physics_state
    iloss_math
    iloss_coordinates
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME DynamicsStateTest COMMAND test_DynamicsState)

add_executable(test_TorqueAggregator test_TorqueAggregator.cpp)
target_link_libraries(test_TorqueAggregator 
    iloss_dynamics
    iloss_math
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME TorqueAggregatorTest COMMAND test_TorqueAggregator)

add_executable(test_GravityGradientTorque test_GravityGradientTorque.cpp)
target_link_libraries(test_GravityGradientTorque 
    iloss_dynamics
    iloss_math
    iloss_constants
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME GravityGradientTorqueTest COMMAND test_GravityGradientTorque)

add_executable(test_DynamicsEngine test_DynamicsEngine.cpp)
target_link_libraries(test_DynamicsEngine 
    iloss_dynamics
    iloss_forces
    iloss_math
    iloss_constants
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME DynamicsEngineTest COMMAND test_DynamicsEngine)

add_executable(test_DynamicsIntegrator test_DynamicsIntegrator.cpp)
target_link_libraries(test_DynamicsIntegrator 
    iloss_dynamics
    iloss_forces
    iloss_integrators
    iloss_math
    iloss_constants
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME DynamicsIntegratorTest COMMAND test_DynamicsIntegrator)

add_executable(test_6DOF_Integration test_6DOF_Integration.cpp)
target_link_libraries(test_6DOF_Integration 
    iloss_dynamics
    iloss_forces
    iloss_integrators
    iloss_math
    iloss_constants
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME 6DOFIntegrationTest COMMAND test_6DOF_Integration)
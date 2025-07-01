# Propulsion tests CMakeLists.txt

# PropellantTank tests
add_executable(test_PropellantTank test_PropellantTank.cpp)
target_link_libraries(test_PropellantTank
    iloss_physics_propulsion
    iloss_dynamics
    iloss_forces
    iloss_physics_state
    iloss_time
    iloss_coordinates
    iloss_constants
    iloss_logging
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_PropellantTank COMMAND test_PropellantTank)

# TODO: Additional tests to be implemented
# - test_Engine.cpp
# - test_ThrustVectorControl.cpp
# - test_PropulsionSystem.cpp
# - test_ThrustForceModel.cpp
# - test_PropulsionIntegration.cpp
# Aerodynamics module tests
set(AERODYNAMICS_TEST_SOURCES
    test_AerodynamicCoefficients.cpp
    test_AerodynamicDatabase.cpp
    test_AerodynamicCalculator.cpp
    test_AerodynamicForceModel.cpp
    test_AerodynamicTorqueModel.cpp
)

# Create test executables
foreach(test_source ${AERODYNAMICS_TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    
    target_link_libraries(${test_name}
        PRIVATE
            iloss_aerodynamics
            iloss_forces
            iloss_physics_state
            iloss_physics_dynamics
            iloss_constants
            iloss_math
            iloss_time
            iloss_coordinates
            iloss_logging
            GTest::gtest
            GTest::gtest_main
    )
    
    # Add to CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
    
    # Set test properties
    set_target_properties(${test_name} PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

# Integration test
add_executable(test_AerodynamicsIntegration test_AerodynamicsIntegration.cpp)

target_link_libraries(test_AerodynamicsIntegration
    PRIVATE
        iloss_aerodynamics
        iloss_forces
        iloss_physics_state
        iloss_physics_dynamics
        iloss_atmosphere
        iloss_math
        iloss_time
        iloss_coordinates
        iloss_logging
        GTest::gtest
        GTest::gtest_main
)

add_test(NAME test_AerodynamicsIntegration COMMAND test_AerodynamicsIntegration)

set_target_properties(test_AerodynamicsIntegration PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)
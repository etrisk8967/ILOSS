# Force model tests
set(FORCE_TEST_SOURCES
    test_ForceModel.cpp
    test_ForceAggregator.cpp
    test_ForceModelRegistry.cpp
)

# Create test executables
foreach(test_source ${FORCE_TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    
    target_link_libraries(${test_name}
        PRIVATE
            iloss_forces
            iloss_physics_state
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

# Add two-body dynamics tests
add_subdirectory(twobody)

# Add gravity model tests
add_subdirectory(gravity)

# Add third-body perturbation tests
add_subdirectory(thirdbody)

# Add drag force tests
add_subdirectory(drag)

# Add solar radiation pressure tests
add_subdirectory(srp)
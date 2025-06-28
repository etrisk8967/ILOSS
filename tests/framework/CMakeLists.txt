# Test Framework CMake Configuration

# Create test framework library
add_library(iloss_test_framework INTERFACE)

target_include_directories(iloss_test_framework INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_test_framework INTERFACE
    GTest::gtest
    GTest::gmock
    iloss_logging
    iloss_config
    iloss_events
    nlohmann_json::nlohmann_json
)

# Add sources that need compilation
add_library(iloss_test_framework_impl STATIC
    fixtures/TestFixtureBase.cpp
)

target_link_libraries(iloss_test_framework_impl
    PUBLIC iloss_test_framework
)

target_compile_definitions(iloss_test_framework_impl PUBLIC BUILD_TESTS)

# Install test framework headers for other projects that might use ILOSS
install(DIRECTORY fixtures generators mocks utilities benchmarks
    DESTINATION include/iloss/test/framework
    FILES_MATCHING PATTERN "*.h"
)

# Create alias for cleaner usage
add_library(iloss::test::framework ALIAS iloss_test_framework)
add_library(iloss::test::framework_impl ALIAS iloss_test_framework_impl)

# Note: Export is commented out for now due to dependencies
# that are not in export sets. This can be re-enabled when
# a full install/export configuration is set up.
# export(TARGETS iloss_test_framework iloss_test_framework_impl
#     NAMESPACE iloss::test::
#     FILE ${CMAKE_BINARY_DIR}/ILOSSTestFrameworkTargets.cmake
# )

# Build framework demo test
if(BUILD_TESTS)
    # Simple demo test
    add_executable(test_framework_simple test_framework_simple.cpp)
    
    target_link_libraries(test_framework_simple
        PRIVATE
            iloss_test_framework_impl
    )
    
    add_test(NAME test_framework_simple COMMAND test_framework_simple)
    set_tests_properties(test_framework_simple PROPERTIES
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        TIMEOUT 30
        LABELS "Framework;Demo"
    )
    
    # Full demo test (requires more dependencies)
    # add_executable(test_framework_demo test_framework_demo.cpp)
    # 
    # target_link_libraries(test_framework_demo
    #     PRIVATE
    #         iloss_test_framework_impl
    #         iloss_math
    #         iloss_time
    # )
    # 
    # add_test(NAME test_framework_demo COMMAND test_framework_demo)
    # set_tests_properties(test_framework_demo PROPERTIES
    #     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    #     TIMEOUT 30
    #     LABELS "Framework;Demo"
    # )
endif()
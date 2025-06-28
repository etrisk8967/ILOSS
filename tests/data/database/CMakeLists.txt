# Database tests CMakeLists.txt

# Test sources
set(DATABASE_TEST_SOURCES
    test_DatabaseManager.cpp
    test_MissionDAO.cpp
    # test_VehicleDAO.cpp      # To be implemented
    # test_LaunchSiteDAO.cpp   # To be implemented
    # test_MigrationManager.cpp # To be implemented
    # test_QueryBuilder.cpp     # To be implemented
)

# Create test executable
foreach(test_source ${DATABASE_TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    
    target_link_libraries(${test_name}
        PRIVATE
            iloss_database
            GTest::gtest
            GTest::gtest_main
            ${SQLite3_LIBRARIES}
    )
    
    target_include_directories(${test_name}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            ${SQLite3_INCLUDE_DIRS}
    )
    
    add_test(NAME ${test_name} COMMAND ${test_name})
    
    # Set test properties
    set_tests_properties(${test_name} PROPERTIES
        TIMEOUT 30
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endforeach()
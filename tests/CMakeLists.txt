# Tests directory CMakeLists.txt

# Add test framework subdirectory
add_subdirectory(framework)

# Add subdirectories for module tests
add_subdirectory(core/math)
add_subdirectory(core/time)
add_subdirectory(core/coordinates)
add_subdirectory(core/constants)
add_subdirectory(core/config)
add_subdirectory(core/logging)
add_subdirectory(core/errors)
add_subdirectory(core/events)
add_subdirectory(core/plugins)

# Add data module tests
add_subdirectory(data)

# Add physics module tests
add_subdirectory(physics)

# Test sources
set(TEST_SOURCES
    test_main.cpp
    # Unit tests will be added as modules are developed
)

# Create test executable
add_executable(iloss_tests ${TEST_SOURCES})

# Set properties
set_target_properties(iloss_tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Include directories
target_include_directories(iloss_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(iloss_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    Eigen3::Eigen
    spdlog::spdlog
    iloss_math
    iloss_time
    iloss_logging
    iloss_errors
)

# Link GeographicLib
if(TARGET GeographicLib::GeographicLib)
    target_link_libraries(iloss_tests PRIVATE GeographicLib::GeographicLib)
else()
    target_link_libraries(iloss_tests PRIVATE GeographicLib)
endif()

# Link SQLite
if(TARGET SQLite::SQLite3)
    target_link_libraries(iloss_tests PRIVATE SQLite::SQLite3)
else()
    target_link_libraries(iloss_tests PRIVATE sqlite3)
endif()

# Link Boost if available
if(Boost_FOUND)
    target_link_libraries(iloss_tests PRIVATE
        Boost::filesystem
        Boost::system
        Boost::date_time
    )
    target_compile_definitions(iloss_tests PRIVATE BOOST_VERSION=${Boost_VERSION})
endif()

# Link GDAL if available
if(GDAL_FOUND)
    target_link_libraries(iloss_tests PRIVATE GDAL::GDAL)
    target_compile_definitions(iloss_tests PRIVATE GDAL_FOUND)
endif()

# Link SPICE if available
if(SPICE_FOUND)
    target_include_directories(iloss_tests PRIVATE ${SPICE_INCLUDE_DIR})
    target_link_libraries(iloss_tests PRIVATE ${SPICE_LIBRARY})
    target_compile_definitions(iloss_tests PRIVATE SPICE_FOUND)
endif()

# Platform-specific libraries
if(ILOSS_PLATFORM_WINDOWS)
    target_link_libraries(iloss_tests PRIVATE ws2_32)
elseif(ILOSS_PLATFORM_LINUX)
    target_link_libraries(iloss_tests PRIVATE pthread dl)
endif()

# Add test
add_test(NAME iloss_tests COMMAND iloss_tests)

# Set test properties
set_tests_properties(iloss_tests PROPERTIES
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    TIMEOUT 60
)

# Library verification test executable
add_executable(test_libraries test_libraries.cpp)

set_target_properties(test_libraries PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(test_libraries PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Link all libraries for testing
target_link_libraries(test_libraries PRIVATE
    GTest::gtest
    Eigen3::Eigen
    spdlog::spdlog
)

# Link GeographicLib
if(TARGET GeographicLib::GeographicLib)
    target_link_libraries(test_libraries PRIVATE GeographicLib::GeographicLib)
else()
    target_link_libraries(test_libraries PRIVATE GeographicLib)
endif()

# Link SQLite
if(TARGET SQLite::SQLite3)
    target_link_libraries(test_libraries PRIVATE SQLite::SQLite3)
else()
    target_link_libraries(test_libraries PRIVATE sqlite3)
endif()

# Link Boost if available
if(Boost_FOUND)
    target_link_libraries(test_libraries PRIVATE
        Boost::filesystem
        Boost::system
        Boost::date_time
    )
    target_compile_definitions(test_libraries PRIVATE BOOST_VERSION=${Boost_VERSION})
endif()

# Link GDAL if available
if(GDAL_FOUND)
    target_link_libraries(test_libraries PRIVATE GDAL::GDAL)
    target_compile_definitions(test_libraries PRIVATE GDAL_FOUND)
endif()

# Link SPICE if available
if(SPICE_FOUND)
    target_include_directories(test_libraries PRIVATE ${SPICE_INCLUDE_DIR})
    target_link_libraries(test_libraries PRIVATE ${SPICE_LIBRARY})
    target_compile_definitions(test_libraries PRIVATE SPICE_FOUND)
endif()

# Platform-specific libraries
if(ILOSS_PLATFORM_WINDOWS)
    target_link_libraries(test_libraries PRIVATE ws2_32)
elseif(ILOSS_PLATFORM_LINUX)
    target_link_libraries(test_libraries PRIVATE pthread dl)
endif()

# Add as a test
add_test(NAME test_libraries COMMAND test_libraries)
set_tests_properties(test_libraries PROPERTIES
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    TIMEOUT 30
)
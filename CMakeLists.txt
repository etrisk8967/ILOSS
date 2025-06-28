cmake_minimum_required(VERSION 3.20)
project(ILOSS 
    VERSION 1.0.0
    DESCRIPTION "Integrated Launch and Orbit Simulation System"
    LANGUAGES CXX C
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add custom module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Platform detection
if(WIN32)
    set(ILOSS_PLATFORM_WINDOWS TRUE)
    add_compile_definitions(ILOSS_PLATFORM_WINDOWS)
elseif(UNIX AND NOT APPLE)
    set(ILOSS_PLATFORM_LINUX TRUE)
    add_compile_definitions(ILOSS_PLATFORM_LINUX)
else()
    message(FATAL_ERROR "Unsupported platform. ILOSS supports Windows and Linux only.")
endif()

# Project options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_DOCS "Build documentation" ON)
option(BUILD_COVERAGE "Enable code coverage" OFF)
option(ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
option(USE_SYSTEM_LIBS "Use system-installed libraries instead of bundled ones" OFF)
option(ENABLE_PROFILING "Enable profiling support" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Set build type specific flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "/MDd /Zi /Ob0 /Od /RTC1 /DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /DNDEBUG")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /Zi /O2 /Ob1 /DNDEBUG")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "/MD /O1 /Ob1 /DNDEBUG")
endif()

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Common warning flags for both C and C++
    add_compile_options(
        -Wall 
        -Wextra 
        -Wpedantic 
        -Wconversion 
        -Wshadow
        -Wcast-align
        -Wunused
        -Wformat=2
        -Wmisleading-indentation
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wnull-dereference
        -Wdouble-promotion
    )
    
    # C++ specific warning flags
    add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>
        $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
        $<$<COMPILE_LANGUAGE:CXX>:-Woverloaded-virtual>
        $<$<COMPILE_LANGUAGE:CXX>:-Wuseless-cast>
    )
    
    # Additional flags for specific compilers
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wnoexcept>)
    endif()
    
    # Profiling support
    if(ENABLE_PROFILING)
        add_compile_options(-pg)
        add_link_options(-pg)
    endif()
    
    # Coverage support
    if(ENABLE_COVERAGE)
        add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
        add_link_options(--coverage)
    endif()
    
    # Sanitizers
    if(ENABLE_SANITIZERS)
        add_compile_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address -fsanitize=undefined)
    endif()
    
elseif(MSVC)
    # MSVC warning flags
    add_compile_options(
        /W4           # Warning level 4
        /permissive-  # Conformance mode
        /Zc:__cplusplus # Enable updated __cplusplus macro
        /utf-8        # Set source and execution character sets to UTF-8
    )
    
    # Disable specific warnings that are too noisy
    add_compile_options(
        /wd4100  # unreferenced formal parameter
        /wd4127  # conditional expression is constant
        /wd4244  # conversion, possible loss of data (handled by explicit casts)
        /wd4267  # size_t to smaller type conversion
    )
    
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(/RTC1)  # Runtime checks
    endif()
    
    # Enable parallel compilation
    add_compile_options(/MP)
    
    # Set warning as error in Release builds
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/WX)
    endif()
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# External dependencies will be added in Task 3 and 4
# Find packages will go here

# Include our custom CMake modules
include(ILOSSConfig)

# Include code coverage support
if(BUILD_COVERAGE)
    include(CodeCoverage)
endif()

# Eigen3 - Linear algebra library
find_package(Eigen3 3.4 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
else()
    message(STATUS "Eigen3 not found. Will use FetchContent to download.")
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
endif()

# Boost - Utility libraries
if(BUILD_SHARED_LIBS)
    set(Boost_USE_STATIC_LIBS OFF)
else()
    set(Boost_USE_STATIC_LIBS ON)
endif()
find_package(Boost 1.75 COMPONENTS system filesystem thread date_time program_options QUIET)
if(Boost_FOUND)
    message(STATUS "Found Boost: ${Boost_VERSION}")
else()
    message(WARNING "Boost not found. Some features may be limited.")
endif()

# SQLite3 - Database
find_package(SQLite3 QUIET)
if(SQLite3_FOUND)
    message(STATUS "Found SQLite3: ${SQLite3_VERSION}")
else()
    message(STATUS "SQLite3 not found. Will use bundled version.")
    add_subdirectory(external)
endif()

# Google Test - Unit testing framework
if(BUILD_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        message(STATUS "Found Google Test: ${GTest_VERSION}")
    else()
        message(STATUS "Google Test not found. Will use FetchContent to download.")
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
            GIT_SHALLOW TRUE
        )
        # For Windows: Prevent overriding the parent project's compiler/linker settings
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()
endif()

# GDAL - Geospatial Data Abstraction Library
find_package(GDAL QUIET)
if(GDAL_FOUND)
    message(STATUS "Found GDAL: ${GDAL_VERSION}")
else()
    message(WARNING "GDAL not found. Terrain and geospatial features will be limited.")
endif()

# GeographicLib - Geodetic calculations
# Try to find GeographicLib with pkg-config first
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GeographicLib QUIET geographiclib)
endif()

if(NOT GeographicLib_FOUND)
    find_package(GeographicLib QUIET)
endif()

if(GeographicLib_FOUND)
    message(STATUS "Found GeographicLib: ${GeographicLib_VERSION}")
else()
    message(STATUS "GeographicLib not found. Will use FetchContent to download.")
    include(FetchContent)
    
    # Save current warning flags
    set(OLD_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(OLD_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    
    # Disable warnings for third-party code
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
    elseif(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /w")
    endif()
    
    FetchContent_Declare(
        geographiclib
        GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git
        GIT_TAG v1.52  # Use older stable version without C++17 issues
        GIT_SHALLOW TRUE
    )
    set(GEOGRAPHICLIB_BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
    set(GEOGRAPHICLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GEOGRAPHICLIB_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(geographiclib)
    
    # Restore warning flags
    set(CMAKE_CXX_FLAGS "${OLD_CMAKE_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${OLD_CMAKE_C_FLAGS}")
endif()

# spdlog - Fast C++ logging library
find_package(spdlog QUIET)
if(spdlog_FOUND)
    message(STATUS "Found spdlog: ${spdlog_VERSION}")
else()
    message(STATUS "spdlog not found. Will use FetchContent to download.")
    include(FetchContent)
    FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.13.0
        GIT_SHALLOW TRUE
    )
    set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(spdlog)
endif()

# nlohmann_json - JSON for Modern C++
find_package(nlohmann_json QUIET)
if(nlohmann_json_FOUND)
    message(STATUS "Found nlohmann_json: ${nlohmann_json_VERSION}")
else()
    message(STATUS "nlohmann_json not found. Will use FetchContent to download.")
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
        GIT_SHALLOW TRUE
    )
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    set(JSON_Install OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# SPICE Toolkit - NASA's ancillary information system
# SPICE requires manual download and setup due to its distribution method
set(SPICE_ROOT_DIR "" CACHE PATH "Path to CSPICE toolkit installation")
if(SPICE_ROOT_DIR)
    find_path(SPICE_INCLUDE_DIR SpiceUsr.h PATHS ${SPICE_ROOT_DIR}/include NO_DEFAULT_PATH)
    find_library(SPICE_LIBRARY 
        NAMES cspice libcspice
        PATHS ${SPICE_ROOT_DIR}/lib
        NO_DEFAULT_PATH
    )
    
    # SPICE uses non-standard library names, check for cspice.a directly
    if(NOT SPICE_LIBRARY AND EXISTS "${SPICE_ROOT_DIR}/lib/cspice.a")
        set(SPICE_LIBRARY "${SPICE_ROOT_DIR}/lib/cspice.a")
    endif()
    
    if(SPICE_INCLUDE_DIR AND SPICE_LIBRARY)
        set(SPICE_FOUND TRUE)
        message(STATUS "Found SPICE toolkit at ${SPICE_ROOT_DIR}")
        message(STATUS "  Include: ${SPICE_INCLUDE_DIR}")
        message(STATUS "  Library: ${SPICE_LIBRARY}")
    else()
        message(WARNING "SPICE toolkit not found at specified location: ${SPICE_ROOT_DIR}")
        if(NOT SPICE_INCLUDE_DIR)
            message(WARNING "  Could not find SpiceUsr.h in ${SPICE_ROOT_DIR}/include")
        endif()
        if(NOT SPICE_LIBRARY)
            message(WARNING "  Could not find cspice library in ${SPICE_ROOT_DIR}/lib")
        endif()
    endif()
else()
    message(STATUS "SPICE toolkit path not specified. Ephemerides features will be limited.")
    message(STATUS "  To enable SPICE, download from https://naif.jpl.nasa.gov/naif/toolkit_C.html")
    message(STATUS "  and set -DSPICE_ROOT_DIR=/path/to/cspice")
endif()

# Qt6 Configuration
set(QT_MIN_VERSION "6.5.0")
find_package(Qt6 ${QT_MIN_VERSION} COMPONENTS Core Widgets OpenGL OpenGLWidgets Concurrent Network QUIET)
if(Qt6_FOUND)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
else()
    message(WARNING "Qt6 (>= ${QT_MIN_VERSION}) not found. UI components will not be built.")
    set(BUILD_UI OFF)
endif()

# osgEarth Configuration
find_package(osgEarth QUIET)
if(osgEarth_FOUND)
    message(STATUS "Found osgEarth: ${OSGEARTH_VERSION}")
    # Find required OSG components
    find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgUtil osgViewer osgGA osgText)
    if(OPENSCENEGRAPH_FOUND)
        message(STATUS "Found OpenSceneGraph: ${OPENSCENEGRAPH_VERSION}")
    endif()
else()
    message(WARNING "osgEarth not found. 3D visualization will not be available.")
    set(BUILD_VISUALIZATION OFF)
endif()

# Set feature flags based on found dependencies
if(NOT DEFINED BUILD_UI)
    set(BUILD_UI ${Qt6_FOUND})
endif()

if(NOT DEFINED BUILD_VISUALIZATION)
    set(BUILD_VISUALIZATION ${osgEarth_FOUND})
endif()

# Add subdirectories
add_subdirectory(src)

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Documentation
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message(WARNING "Doxygen not found, documentation will not be built")
    endif()
endif()

# Installation rules
install(DIRECTORY include/ DESTINATION include)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ILOSSConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Summary
message(STATUS "")
message(STATUS "ILOSS Configuration Summary")
message(STATUS "===========================")
message(STATUS "Version:          ${PROJECT_VERSION}")
message(STATUS "Platform:         ${CMAKE_SYSTEM_NAME}")
message(STATUS "Architecture:     ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler:         ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Features:")
message(STATUS "  Build tests:      ${BUILD_TESTS}")
message(STATUS "  Build docs:       ${BUILD_DOCS}")
message(STATUS "  Build UI:         ${BUILD_UI}")
message(STATUS "  3D Visualization: ${BUILD_VISUALIZATION}")
message(STATUS "  Coverage:         ${ENABLE_COVERAGE}")
message(STATUS "  Sanitizers:       ${ENABLE_SANITIZERS}")
message(STATUS "  Profiling:        ${ENABLE_PROFILING}")
message(STATUS "  Shared libs:      ${BUILD_SHARED_LIBS}")
message(STATUS "")
message(STATUS "Dependencies:")
if(Eigen3_FOUND)
    message(STATUS "  Eigen3:         ${EIGEN3_VERSION}")
else()
    message(STATUS "  Eigen3:         (bundled)")
endif()
if(Boost_FOUND)
    message(STATUS "  Boost:          ${Boost_VERSION}")
endif()
if(SQLite3_FOUND AND NOT TARGET sqlite3)
    message(STATUS "  SQLite3:        ${SQLite3_VERSION}")
else()
    message(STATUS "  SQLite3:        (bundled)")
endif()
if(BUILD_TESTS)
    if(GTest_FOUND)
        message(STATUS "  Google Test:    ${GTest_VERSION}")
    else()
        message(STATUS "  Google Test:    (bundled)")
    endif()
endif()
if(GDAL_FOUND)
    message(STATUS "  GDAL:           ${GDAL_VERSION}")
endif()
if(GeographicLib_FOUND)
    message(STATUS "  GeographicLib:  ${GeographicLib_VERSION}")
else()
    message(STATUS "  GeographicLib:  (bundled)")
endif()
if(spdlog_FOUND)
    message(STATUS "  spdlog:         ${spdlog_VERSION}")
else()
    message(STATUS "  spdlog:         (bundled)")
endif()
if(SPICE_FOUND)
    message(STATUS "  SPICE:          Found")
endif()
if(Qt6_FOUND)
    message(STATUS "  Qt6:            ${Qt6_VERSION}")
endif()
if(osgEarth_FOUND)
    message(STATUS "  osgEarth:       ${OSGEARTH_VERSION}")
    message(STATUS "  OpenSceneGraph: ${OPENSCENEGRAPH_VERSION}")
endif()
message(STATUS "")

# Enable code coverage if requested
if(BUILD_COVERAGE AND BUILD_TESTS)
    enable_coverage_for_tests()
    message(STATUS "Code coverage enabled. Run 'make coverage' after building to generate report.")
endif()
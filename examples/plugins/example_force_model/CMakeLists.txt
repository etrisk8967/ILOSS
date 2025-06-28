cmake_minimum_required(VERSION 3.20)
project(ExampleForceModelPlugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the main project include directory
set(ILOSS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include")

# Create the plugin library
add_library(example_force_model SHARED
    ExampleForceModelPlugin.cpp
    ExampleForceModelPlugin.h
)

# Include directories
target_include_directories(example_force_model PRIVATE
    ${ILOSS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set properties for plugin
set_target_properties(example_force_model PROPERTIES
    PREFIX ""  # Remove lib prefix on Linux
    POSITION_INDEPENDENT_CODE ON
)

# Platform-specific settings
if(WIN32)
    set_target_properties(example_force_model PROPERTIES
        SUFFIX ".dll"
    )
elseif(APPLE)
    set_target_properties(example_force_model PROPERTIES
        SUFFIX ".dylib"
    )
else()
    set_target_properties(example_force_model PROPERTIES
        SUFFIX ".so"
    )
endif()

# Installation
install(TARGETS example_force_model
    LIBRARY DESTINATION plugins
    RUNTIME DESTINATION plugins
)
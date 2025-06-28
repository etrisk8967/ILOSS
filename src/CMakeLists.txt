# Source directory CMakeLists.txt

# Add subdirectories for core modules
add_subdirectory(core/math)
add_subdirectory(core/time)
add_subdirectory(core/coordinates)
add_subdirectory(core/constants)
add_subdirectory(core/config)
add_subdirectory(core/logging)
add_subdirectory(core/errors)
add_subdirectory(core/events)
add_subdirectory(core/plugins)

# Add data modules
add_subdirectory(data)

# Add physics modules
add_subdirectory(physics)

# Collect source files
set(ILOSS_SOURCES
    # Core sources will be added here as modules are developed
)

# Create main library (will be populated in future tasks)
# add_library(iloss_core ${ILOSS_SOURCES})

# Main executable placeholder
add_executable(iloss main.cpp)

# Link libraries
target_link_libraries(iloss 
    PRIVATE 
    iloss_math
    iloss_time
    iloss_logging
    iloss_errors
)
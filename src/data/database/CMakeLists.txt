# Database module CMakeLists.txt

# Source files
set(DATABASE_SOURCES
    DatabaseManager.cpp
    MissionDAO.cpp
    # VehicleDAO.cpp      # To be implemented
    # LaunchSiteDAO.cpp   # To be implemented
    # MigrationManager.cpp # To be implemented
    # QueryBuilder.cpp     # To be implemented
)

# Create database library
add_library(iloss_database STATIC ${DATABASE_SOURCES})

# Include directories
target_include_directories(iloss_database 
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/include
        ${SQLite3_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies
target_link_libraries(iloss_database
    PUBLIC
        iloss_logging
        ${SQLite3_LIBRARIES}
)

# Set properties
set_target_properties(iloss_database PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
# ILOSSConfig.cmake - Project configuration helpers

# Function to check and report missing dependencies
function(iloss_check_required_dependency DEP_NAME DEP_FOUND)
    if(NOT ${DEP_FOUND})
        message(FATAL_ERROR "${DEP_NAME} is required but not found. Please install it or set the appropriate CMake variables.")
    endif()
endfunction()

# Function to find or fetch a dependency
function(iloss_find_or_fetch DEP_NAME)
    # This will be expanded in Task 3/4 to handle fetching dependencies
    # For now, just try to find the package
    find_package(${DEP_NAME} ${ARGN})
endfunction()

# Set up installation paths
if(WIN32)
    set(ILOSS_INSTALL_BIN_DIR "bin")
    set(ILOSS_INSTALL_LIB_DIR "lib")
    set(ILOSS_INSTALL_INCLUDE_DIR "include")
    set(ILOSS_INSTALL_DATA_DIR "data")
    set(ILOSS_INSTALL_DOC_DIR "doc")
else()
    include(GNUInstallDirs)
    set(ILOSS_INSTALL_BIN_DIR ${CMAKE_INSTALL_BINDIR})
    set(ILOSS_INSTALL_LIB_DIR ${CMAKE_INSTALL_LIBDIR})
    set(ILOSS_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})
    set(ILOSS_INSTALL_DATA_DIR ${CMAKE_INSTALL_DATADIR}/iloss)
    set(ILOSS_INSTALL_DOC_DIR ${CMAKE_INSTALL_DOCDIR})
endif()

# Helper function to add an ILOSS module
function(iloss_add_module MODULE_NAME)
    set(options SHARED STATIC)
    set(oneValueArgs TYPE)
    set(multiValueArgs SOURCES PUBLIC_HEADERS PRIVATE_HEADERS DEPENDENCIES)
    cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # Create library
    if(MODULE_SHARED OR BUILD_SHARED_LIBS)
        add_library(${MODULE_NAME} SHARED ${MODULE_SOURCES})
    else()
        add_library(${MODULE_NAME} STATIC ${MODULE_SOURCES})
    endif()
    
    # Set properties
    set_target_properties(${MODULE_NAME} PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    
    # Link dependencies
    if(MODULE_DEPENDENCIES)
        target_link_libraries(${MODULE_NAME} PUBLIC ${MODULE_DEPENDENCIES})
    endif()
    
    # Install rules
    install(TARGETS ${MODULE_NAME}
        RUNTIME DESTINATION ${ILOSS_INSTALL_BIN_DIR}
        LIBRARY DESTINATION ${ILOSS_INSTALL_LIB_DIR}
        ARCHIVE DESTINATION ${ILOSS_INSTALL_LIB_DIR}
    )
    
    if(MODULE_PUBLIC_HEADERS)
        install(FILES ${MODULE_PUBLIC_HEADERS}
            DESTINATION ${ILOSS_INSTALL_INCLUDE_DIR}/iloss
        )
    endif()
endfunction()
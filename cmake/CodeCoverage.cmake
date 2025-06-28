# Code coverage configuration for ILOSS
#
# This module provides functions to enable code coverage analysis
# Supports both GCC/Clang (gcov/llvm-cov) and MSVC coverage tools

include(CMakeParseArguments)

# Check if coverage is supported on this platform/compiler
function(coverage_supported VAR)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(${VAR} TRUE PARENT_SCOPE)
    elseif(MSVC)
        # MSVC coverage requires VS 2019 or later
        if(MSVC_VERSION GREATER_EQUAL 1920)
            set(${VAR} TRUE PARENT_SCOPE)
        else()
            set(${VAR} FALSE PARENT_SCOPE)
        endif()
    else()
        set(${VAR} FALSE PARENT_SCOPE)
    endif()
endfunction()

# Enable coverage for a target
function(enable_coverage TARGET)
    coverage_supported(COVERAGE_SUPPORTED)
    
    if(NOT COVERAGE_SUPPORTED)
        message(WARNING "Code coverage not supported for ${CMAKE_CXX_COMPILER_ID}")
        return()
    endif()
    
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # GCC/Clang coverage flags
        target_compile_options(${TARGET} PRIVATE
            --coverage
            -fprofile-arcs
            -ftest-coverage
        )
        
        target_link_options(${TARGET} PRIVATE
            --coverage
            -fprofile-arcs
            -ftest-coverage
        )
        
        # Add gcov/llvm-cov note files to clean target
        set_property(DIRECTORY APPEND PROPERTY
            ADDITIONAL_CLEAN_FILES
            "${CMAKE_CURRENT_BINARY_DIR}/*.gcda"
            "${CMAKE_CURRENT_BINARY_DIR}/*.gcno"
            "${CMAKE_CURRENT_BINARY_DIR}/*.profraw"
            "${CMAKE_CURRENT_BINARY_DIR}/*.profdata"
        )
        
    elseif(MSVC)
        # MSVC coverage flags
        target_compile_options(${TARGET} PRIVATE
            /fsanitize=coverage
        )
        
        target_link_options(${TARGET} PRIVATE
            /PROFILE
        )
    endif()
endfunction()

# Create coverage report target
function(create_coverage_target)
    cmake_parse_arguments(COV
        ""
        "NAME;OUTPUT_DIR"
        "DEPENDENCIES;EXCLUDE_PATTERNS"
        ${ARGN}
    )
    
    if(NOT COV_NAME)
        set(COV_NAME "coverage")
    endif()
    
    if(NOT COV_OUTPUT_DIR)
        set(COV_OUTPUT_DIR "${CMAKE_BINARY_DIR}/coverage")
    endif()
    
    coverage_supported(COVERAGE_SUPPORTED)
    if(NOT COVERAGE_SUPPORTED)
        message(STATUS "Coverage reporting not available")
        return()
    endif()
    
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # Find coverage tools
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
            find_program(GCOV_EXECUTABLE gcov)
            set(COV_TOOL ${GCOV_EXECUTABLE})
        else()
            find_program(LLVM_COV_EXECUTABLE llvm-cov)
            set(COV_TOOL ${LLVM_COV_EXECUTABLE})
        endif()
        
        find_program(LCOV_EXECUTABLE lcov)
        find_program(GENHTML_EXECUTABLE genhtml)
        
        if(NOT COV_TOOL)
            message(WARNING "Coverage tool (gcov/llvm-cov) not found")
            return()
        endif()
        
        if(NOT LCOV_EXECUTABLE OR NOT GENHTML_EXECUTABLE)
            message(WARNING "lcov/genhtml not found - install lcov for HTML reports")
            
            # Create simple text coverage report
            add_custom_target(${COV_NAME}
                COMMAND ${CMAKE_COMMAND} -E make_directory ${COV_OUTPUT_DIR}
                COMMAND ${CMAKE_COMMAND} -E echo "Running tests..."
                COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
                COMMAND ${CMAKE_COMMAND} -E echo "Generating coverage report..."
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                DEPENDS ${COV_DEPENDENCIES}
                COMMENT "Generating code coverage report"
            )
        else()
            # Create full HTML coverage report with lcov
            set(COVERAGE_INFO "${COV_OUTPUT_DIR}/coverage.info")
            set(COVERAGE_CLEANED "${COV_OUTPUT_DIR}/coverage_cleaned.info")
            
            # Build exclude patterns
            set(LCOV_EXCLUDES)
            foreach(PATTERN ${COV_EXCLUDE_PATTERNS})
                list(APPEND LCOV_EXCLUDES "--exclude" "${PATTERN}")
            endforeach()
            
            # Default excludes
            list(APPEND LCOV_EXCLUDES
                "--exclude" "*/tests/*"
                "--exclude" "*/external/*"
                "--exclude" "*/build/*"
                "--exclude" "/usr/*"
                "--exclude" "*/gtest/*"
                "--exclude" "*/gmock/*"
            )
            
            add_custom_target(${COV_NAME}
                # Clean previous coverage data
                COMMAND ${LCOV_EXECUTABLE} --directory . --zerocounters
                
                # Run tests
                COMMAND ${CMAKE_COMMAND} -E echo "Running tests..."
                COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
                
                # Capture coverage data
                COMMAND ${CMAKE_COMMAND} -E echo "Capturing coverage data..."
                COMMAND ${LCOV_EXECUTABLE} --directory . --capture --output-file ${COVERAGE_INFO}
                
                # Remove excluded files
                COMMAND ${LCOV_EXECUTABLE} --remove ${COVERAGE_INFO} ${LCOV_EXCLUDES} --output-file ${COVERAGE_CLEANED}
                
                # Generate HTML report
                COMMAND ${CMAKE_COMMAND} -E echo "Generating HTML report..."
                COMMAND ${GENHTML_EXECUTABLE} ${COVERAGE_CLEANED} --output-directory ${COV_OUTPUT_DIR}/html
                
                # Print summary
                COMMAND ${LCOV_EXECUTABLE} --list ${COVERAGE_CLEANED}
                
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                DEPENDS ${COV_DEPENDENCIES}
                COMMENT "Generating code coverage report"
            )
            
            add_custom_command(TARGET ${COV_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E echo "Coverage report generated at: ${COV_OUTPUT_DIR}/html/index.html"
            )
        endif()
        
    elseif(MSVC)
        # MSVC coverage with vstest.console
        find_program(VSTEST_EXECUTABLE vstest.console
            HINTS "C:/Program Files/Microsoft Visual Studio/2022/*/Common7/IDE/Extensions/TestPlatform"
                  "C:/Program Files (x86)/Microsoft Visual Studio/2019/*/Common7/IDE/Extensions/TestPlatform"
        )
        
        if(NOT VSTEST_EXECUTABLE)
            message(WARNING "vstest.console.exe not found")
            return()
        endif()
        
        add_custom_target(${COV_NAME}
            COMMAND ${CMAKE_COMMAND} -E echo "Running tests with coverage..."
            COMMAND ${VSTEST_EXECUTABLE} "$<TARGET_FILE_DIR:${COV_DEPENDENCIES}>" /EnableCodeCoverage
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            DEPENDS ${COV_DEPENDENCIES}
            COMMENT "Generating code coverage report"
        )
    endif()
endfunction()

# Function to generate coverage badge
function(generate_coverage_badge)
    cmake_parse_arguments(BADGE
        ""
        "OUTPUT;COVERAGE_FILE"
        ""
        ${ARGN}
    )
    
    if(NOT BADGE_OUTPUT)
        set(BADGE_OUTPUT "${CMAKE_BINARY_DIR}/coverage_badge.svg")
    endif()
    
    if(NOT BADGE_COVERAGE_FILE)
        set(BADGE_COVERAGE_FILE "${CMAKE_BINARY_DIR}/coverage/coverage_cleaned.info")
    endif()
    
    # Create script to generate badge
    file(WRITE "${CMAKE_BINARY_DIR}/generate_coverage_badge.cmake"
        "if(EXISTS \"${BADGE_COVERAGE_FILE}\")
    file(READ \"${BADGE_COVERAGE_FILE}\" COVERAGE_DATA)
    string(REGEX MATCH \"lines\\\\.\\\\.+: ([0-9]+\\\\.[0-9]+)%\" COVERAGE_MATCH \"\${COVERAGE_DATA}\")
    if(CMAKE_MATCH_1)
        set(COVERAGE_PERCENT \${CMAKE_MATCH_1})
        
        # Determine color based on coverage
        if(COVERAGE_PERCENT GREATER_EQUAL 90)
            set(COLOR \"brightgreen\")
        elseif(COVERAGE_PERCENT GREATER_EQUAL 80)
            set(COLOR \"green\")
        elseif(COVERAGE_PERCENT GREATER_EQUAL 70)
            set(COLOR \"yellowgreen\")
        elseif(COVERAGE_PERCENT GREATER_EQUAL 60)
            set(COLOR \"yellow\")
        elseif(COVERAGE_PERCENT GREATER_EQUAL 50)
            set(COLOR \"orange\")
        else()
            set(COLOR \"red\")
        endif()
        
        # Generate SVG badge
        file(WRITE \"${BADGE_OUTPUT}\"
            \"<svg xmlns='http://www.w3.org/2000/svg' width='114' height='20'>
    <linearGradient id='b' x2='0' y2='100%'>
        <stop offset='0' stop-color='#bbb' stop-opacity='.1'/>
        <stop offset='1' stop-opacity='.1'/>
    </linearGradient>
    <clipPath id='a'>
        <rect width='114' height='20' rx='3' fill='#fff'/>
    </clipPath>
    <g clip-path='url(#a)'>
        <path fill='#555' d='M0 0h63v20H0z'/>
        <path fill='\${COLOR}' d='M63 0h51v20H63z'/>
        <path fill='url(#b)' d='M0 0h114v20H0z'/>
    </g>
    <g fill='#fff' text-anchor='middle' font-family='DejaVu Sans,Verdana,Geneva,sans-serif' font-size='11'>
        <text x='31.5' y='15' fill='#010101' fill-opacity='.3'>coverage</text>
        <text x='31.5' y='14'>coverage</text>
        <text x='87.5' y='15' fill='#010101' fill-opacity='.3'>\${COVERAGE_PERCENT}%</text>
        <text x='87.5' y='14'>\${COVERAGE_PERCENT}%</text>
    </g>
</svg>\")
        message(STATUS \"Coverage: \${COVERAGE_PERCENT}%\")
    endif()
endif()"
    )
    
    add_custom_target(coverage-badge
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_BINARY_DIR}/generate_coverage_badge.cmake"
        DEPENDS coverage
        COMMENT "Generating coverage badge"
    )
endfunction()

# Enable coverage for all test targets
function(enable_coverage_for_tests)
    if(NOT BUILD_COVERAGE)
        return()
    endif()
    
    # Get all test targets
    get_property(TEST_TARGETS DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY TESTS)
    
    foreach(TEST ${TEST_TARGETS})
        # Enable coverage for each test executable
        if(TARGET ${TEST})
            enable_coverage(${TEST})
        endif()
    endforeach()
    
    # Create main coverage target
    create_coverage_target(
        NAME coverage
        OUTPUT_DIR "${CMAKE_BINARY_DIR}/coverage"
        EXCLUDE_PATTERNS
            "*/tests/*"
            "*/external/*"
            "*/build/*"
            "/usr/*"
    )
    
    # Generate coverage badge
    generate_coverage_badge()
endfunction()
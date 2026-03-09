# CodeCoverage.cmake
# CMake module for code coverage configuration
# Usage in CMakeLists.txt:
#   include(CodeCoverage)
#   target_enable_coverage(my_target)

# Find required programs
find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)
find_program(GCOVR_PATH gcovr)

# Check if coverage is enabled
if(CMAKE_BUILD_TYPE STREQUAL "Coverage" OR ENABLE_COVERAGE)
    message(STATUS "Code coverage enabled")

    # Set coverage flags
    set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C++ compiler during coverage builds")
    set(CMAKE_C_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C compiler during coverage builds")
    set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "--coverage"
        CACHE STRING "Flags used for linking binaries during coverage builds")
    set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "--coverage"
        CACHE STRING "Flags used by the shared libraries linker during coverage builds")

    mark_as_advanced(
        CMAKE_CXX_FLAGS_COVERAGE
        CMAKE_C_FLAGS_COVERAGE
        CMAKE_EXE_LINKER_FLAGS_COVERAGE
        CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    )
endif()

# Function to enable coverage for a target
function(target_enable_coverage target)
    if(CMAKE_BUILD_TYPE STREQUAL "Coverage" OR ENABLE_COVERAGE)
        message(STATUS "Enabling coverage for target: ${target}")

        # Add coverage compile options
        target_compile_options(${target} PRIVATE
            --coverage
            -fprofile-arcs
            -ftest-coverage
        )

        # Add coverage link options
        target_link_options(${target} PRIVATE
            --coverage
        )

        # Add coverage definitions
        target_compile_definitions(${target} PRIVATE
            COVERAGE_ENABLED=1
        )
    endif()
endfunction()

# Function to add coverage target for a package
function(add_coverage_target)
    if(CMAKE_BUILD_TYPE STREQUAL "Coverage" OR ENABLE_COVERAGE)
        if(LCOV_PATH AND GENHTML_PATH)
            # Add custom target for generating coverage report
            add_custom_target(${PROJECT_NAME}_coverage
                COMMAND ${LCOV_PATH} --capture --directory ${CMAKE_BINARY_DIR} --output-file coverage.info
                COMMAND ${LCOV_PATH} --remove coverage.info '/usr/*' '/opt/*' '*/test/*' --output-file coverage.info
                COMMAND ${GENHTML_PATH} coverage.info --output-directory coverage-report
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Generating coverage report for ${PROJECT_NAME}"
            )
        endif()
    endif()
endfunction()

# Print status
message(STATUS "CodeCoverage.cmake loaded")
message(STATUS "  lcov: ${LCOV_PATH}")
message(STATUS "  genhtml: ${GENHTML_PATH}")
message(STATUS "  gcovr: ${GCOVR_PATH}")

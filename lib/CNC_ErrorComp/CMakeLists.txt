cmake_minimum_required(VERSION 3.16)
project(CNC_ErrorComp VERSION 1.0.0 LANGUAGES C CXX)

# ============================================================
# Global settings
# ============================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type (Debug / Release / RelWithDebInfo / MinSizeRel)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# Static library: CNC_ErrorComp (pure C, no external dependency)
# ============================================================
add_library(CNC_ErrorComp STATIC
    src/backlash_comp.c
    src/perp_comp.c
    src/pitch_comp.c
    src/pitch_comp_table_init.c
)

target_include_directories(CNC_ErrorComp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(CNC_ErrorComp PUBLIC c_std_11)

# Output static library to project root lib/
set_target_properties(CNC_ErrorComp PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_compile_options(CNC_ErrorComp PRIVATE
    $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>>:
        -Wall -Wextra -Wpedantic>
    $<$<C_COMPILER_ID:MSVC>:
        /W4>
)

# ============================================================
# Install rules
# ============================================================
include(GNUInstallDirs)

install(TARGETS CNC_ErrorComp
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# ============================================================
# Tests
# ============================================================
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

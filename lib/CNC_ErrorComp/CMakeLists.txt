cmake_minimum_required(VERSION 3.16)
project(CNC_ErrorComp LANGUAGES C CXX)

set(CMAKE_AUTOMOC ON)

# ============================================================
# Static library: CNC_ErrorComp (pure C, no Qt dependency)
# ============================================================
add_library(CNC_ErrorComp STATIC
    src/perp_comp.c
    src/pitch_comp.c
    src/pitch_comp_table_init.c
)

target_include_directories(CNC_ErrorComp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(CNC_ErrorComp PUBLIC c_std_11)

set_target_properties(CNC_ErrorComp PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
)

# ============================================================
# Unit tests (require Qt Test)
# ============================================================
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    # Support both Qt5 and Qt6
    find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Test)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test)

    # ---- perp_comp test ----
    add_executable(tst_perp_comp
        tests/unit_perp_comp/tst_perp.cpp
    )

    target_link_libraries(tst_perp_comp PRIVATE
        CNC_ErrorComp
        Qt${QT_VERSION_MAJOR}::Test
    )

    target_include_directories(tst_perp_comp PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    add_test(NAME perp_comp COMMAND tst_perp_comp)

    # ---- pitch_comp test ----
    add_executable(tst_pitch_comp
        tests/unit_pitch_comp/tst_pitch.cpp
    )

    target_link_libraries(tst_pitch_comp PRIVATE
        CNC_ErrorComp
        Qt${QT_VERSION_MAJOR}::Test
    )

    target_include_directories(tst_pitch_comp PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    add_test(NAME pitch_comp COMMAND tst_pitch_comp)
endif()

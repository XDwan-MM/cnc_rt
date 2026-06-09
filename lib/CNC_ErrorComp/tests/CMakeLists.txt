find_package(GTest REQUIRED)

# Common test compilation options
function(add_comp_test TARGET_NAME SOURCE_FILE)
    add_executable(${TARGET_NAME} ${SOURCE_FILE})

    target_link_libraries(${TARGET_NAME} PRIVATE
        CNC_ErrorComp
        GTest::gtest_main
    )

    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )

    target_compile_options(${TARGET_NAME} PRIVATE
        $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:
            -Wall -Wextra -Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:
            /W4>
    )

    gtest_discover_tests(${TARGET_NAME})
endfunction()

# ---- perp_comp test ----
add_comp_test(tst_perp_comp test_perp_comp.cpp)

# ---- pitch_comp test ----
add_comp_test(tst_pitch_comp test_pitch_comp.cpp)

# ---- backlash_comp test ----
add_comp_test(tst_backlash_comp test_backlash_comp.cpp)

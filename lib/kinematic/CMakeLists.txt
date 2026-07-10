cmake_minimum_required(VERSION 3.10)
project(kinematic C CXX)

# set(CMAKE_BUILD_TYPE Debug)

# set(VERSION "1.11.0")

set(CNC_RT_ROOT "" CACHE PATH "Path to cnc_rt root")
if(NOT CNC_RT_ROOT)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../src/utils/math/cnc_math.c")
        get_filename_component(CNC_RT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../cnc_rt/src/utils/math/cnc_math.c")
        get_filename_component(CNC_RT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../cnc_rt" ABSOLUTE)
    else()
        message(FATAL_ERROR "Cannot locate cnc_rt root. Set CNC_RT_ROOT to the cnc_rt directory.")
    endif()
endif()

set(CNC_RT_SRC_DIR "${CNC_RT_ROOT}/src")
set(CNC_RT_LIB_DIR "${CNC_RT_ROOT}/lib")

find_package(Threads REQUIRED)

# 创建 kinematic 静态库
add_library(kinematic STATIC
    trajectory.c
    ${CNC_RT_SRC_DIR}/utils/math/cnc_math.c
    ${CNC_RT_SRC_DIR}/datadef/plan_queue.c
)

# 设置公共头文件目录
target_include_directories(kinematic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CNC_RT_SRC_DIR}/datadef
    ${CNC_RT_SRC_DIR}/motion
    ${CNC_RT_SRC_DIR}/utils/math
    ${CNC_RT_LIB_DIR}/cncshm/api
    ${CNC_RT_LIB_DIR}/parser/api
)

target_link_libraries(kinematic PUBLIC m Threads::Threads)

# 设置库的输出目录和名称
set_target_properties(kinematic PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    OUTPUT_NAME "kinematic"
)

option(BUILD_TRAJECTORY_TESTS "Build trajectory unit tests" OFF)
if(BUILD_TRAJECTORY_TESTS)
    enable_testing()

    add_executable(trajectory_tests
        test/main.cpp
        test/tst_trajectory.cpp
    )
    target_compile_features(trajectory_tests PRIVATE cxx_std_17)
    target_include_directories(trajectory_tests PRIVATE
        ${CNC_RT_ROOT}/test/gtest_compat
    )
    target_link_libraries(trajectory_tests PRIVATE kinematic)

    add_test(NAME trajectory_tests COMMAND trajectory_tests)
endif()

add_custom_target(run_test
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/run_test.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    USES_TERMINAL
    COMMENT "Build and run trajectory tests via test/run_test.sh"
)

cmake_minimum_required(VERSION 3.10)
project(circleSmooth C CXX)

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

# 创建 circleSmooth 静态库
add_library(circleSmooth STATIC
    circle_smooth.c
    ${CNC_RT_SRC_DIR}/utils/math/cnc_math.c
    ${CNC_RT_SRC_DIR}/datadef/plan_queue.c
)

# 设置公共头文件目录
target_include_directories(circleSmooth PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CNC_RT_SRC_DIR}/datadef
    ${CNC_RT_SRC_DIR}/protect
    ${CNC_RT_SRC_DIR}/motion
    ${CNC_RT_SRC_DIR}/motion/rerun
    ${CNC_RT_SRC_DIR}/motion/auto
    ${CNC_RT_SRC_DIR}/motion/auto/pathplan
    ${CNC_RT_SRC_DIR}/motion/auto/trajplan
    ${CNC_RT_SRC_DIR}/motion/manual
    ${CNC_RT_SRC_DIR}/motion/manual/jog
    ${CNC_RT_SRC_DIR}/hal
    ${CNC_RT_SRC_DIR}/plc/plc_scanner
    ${CNC_RT_SRC_DIR}/utils/math
    
    ${CNC_RT_LIB_DIR}/cncshm/api
    ${CNC_RT_LIB_DIR}/parser/api
    ${CNC_RT_LIB_DIR}/kinematic/include
    /opt/GreeMaster/include/libGREEMASTER
    "/home/pub/Qt5.15.16/5.15.16/gcc_64/include/QtCore",
    "/home/pub/Qt5.15.16/5.15.16/gcc_64/include",
    "/home/pub/Qt5.15.16/5.15.16/gcc_64/bin"

)

target_link_libraries(circleSmooth PUBLIC m Threads::Threads)

# 设置库的输出目录和名称
set_target_properties(circleSmooth PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    OUTPUT_NAME "circleSmooth"
)

option(BUILD_CIRCLESMOOTH_TESTS "Build circleSmooth unit tests" OFF)
if(BUILD_CIRCLESMOOTH_TESTS)
    enable_testing()

    add_executable(circleSmooth_tests
        test/main.cpp
        # test/tst_trajectory.cpp
    )
    target_compile_features(circleSmooth_tests PRIVATE cxx_std_17)
    target_include_directories(circleSmooth_tests PRIVATE
        ${CNC_RT_ROOT}/test/gtest_compat
    )
    target_link_libraries(circleSmooth_tests PRIVATE nurbs)

    add_test(NAME circleSmooth_tests COMMAND circleSmooth_tests)
endif()

add_custom_target(run_test
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/run_test.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    USES_TERMINAL
    COMMENT "Build and run circleSmooth tests via test/run_test.sh"
)

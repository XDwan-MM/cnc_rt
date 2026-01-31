cmake_minimum_required(VERSION 3.10)
project(kinematic C)

# 创建 kinematic 静态库
add_library(kinematic STATIC
    trajectory.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/utils/math/cnc_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/datadef/plan_queue.c
)

# 设置公共头文件目录
target_include_directories(kinematic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/datadef
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/motion
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/utils/math
    ${CMAKE_CURRENT_SOURCE_DIR}/../cncshm/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../parser/api
)

# 设置库的输出目录和名称
set_target_properties(kinematic PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    OUTPUT_NAME "kinematic"
)

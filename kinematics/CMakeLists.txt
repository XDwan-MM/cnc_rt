# 创建 kinematics 静态库
add_library(kinematics STATIC
    trajectory.c
    trajectory.h
    # 添加依赖的模块源文件
    ${CMAKE_SOURCE_DIR}/src/math/cnc_math.c
    ${CMAKE_SOURCE_DIR}/src/math/cnc_math.h
    ${CMAKE_SOURCE_DIR}/src/datadef/common.c
    ${CMAKE_SOURCE_DIR}/src/datadef/common.h
    ${CMAKE_SOURCE_DIR}/src/datadef/plan_queue.c
    ${CMAKE_SOURCE_DIR}/src/datadef/plan_queue.h
    ${CMAKE_SOURCE_DIR}/src/datadef/gcode_queue.c
    ${CMAKE_SOURCE_DIR}/src/datadef/gcode_queue.h
)

# 设置公共头文件目录
target_include_directories(kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# 设置库的输出目录和名称
set_target_properties(kinematics PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    # 设置静态库输出名称（不带 lib 前缀）
    PREFIX ""
    OUTPUT_NAME "kinematics"
)

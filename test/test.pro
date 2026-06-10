QT += testlib
QT += core

CONFIG += qt console warn_on depend_includepath testcase
CONFIG -= app_bundle

DEFINES += PLATFORM_1
DEFINES += MASTER_RUN_ON

TEMPLATE = app

TARGET = tests

# 测试注册头所在目录（test_registry.h / main.cpp）
INCLUDEPATH += $$PWD

# 添加共享内存库
include(lib/cncshm/api/apishm.pri)
LIBS += -L"$$PWD/../lib/cncshm/bin/" -lshm
INCLUDEPATH += $$PWD/../lib/cncshm/api $$PWD/../lib/cncshm

# 添加译码
LIBS += -L$$PWD/../lib/parser/lib/ -lcnc_parser
INCLUDEPATH += $$PWD/../lib/parser/api $$PWD/../lib/parser

# 添加运动库
LIBS += -L"$$PWD/../lib/kinematic/bin/" -lkinematic
INCLUDEPATH += $$PWD/../lib/kinematic/include $$PWD/../lib/kinematic

# 添加误差补偿库
LIBS += -L"$$PWD/../lib/CNC_ErrorComp/bin" -lCNC_ErrorComp
INCLUDEPATH += $$PWD/../lib/CNC_ErrorComp/include $$PWD/../lib/CNC_ErrorComp

unix:!macx: {
    DESTDIR = $$PWD/bin
}

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -Wl,--no-as-needed -lxml2 -Wl,--as-needed
LIBS += -L "/usr/local/lib/" -lethercat
INCLUDEPATH += /opt/GreeMaster/include/libGREEMASTER/

HEADERS += \
    ../src/common/debug_mode.h \
    ../src/compensation/comp_data.h \
    ../src/datadef/datatype.h \
    ../src/datadef/event_queue.h \
    ../src/datadef/eventdef.h \
    ../src/datadef/gcode_queue.h \
    ../src/datadef/global.h \
    ../src/datadef/rt_data.h \
    ../src/datadef/mcode_queue.h \
    ../src/datadef/plan_queue.h \
    ../src/datadef/rt_data.h \
    ../src/hal/hal_driver.h \
    ../src/hal/hal_io.h \
    ../src/master/master_sim.h \
    ../src/master/master.h \
    ../src/motion/auto/interpolation.h \
    ../src/motion/auto/mem.h \
    ../src/motion/auto/mem_mst.h \
    ../src/motion/auto/mem_gcode.h \
    ../src/motion/auto/parser.h \
    ../src/motion/auto/pathplan.h \
    ../src/motion/spindle/spl_control.h \
    ../src/motion/alarm.h \
    ../src/motion/egear_ratio.h \
    ../src/motion/emcmot.h \
    ../src/motion/ferror_control.h \
    ../src/motion/handwheel.h \
    ../src/motion/homing.h \
    ../src/motion/jog.h \
    ../src/motion/jog_planner.h \
    ../src/motion/stored_stroke_check.h \
    ../src/plc/pmc_a_input_scanner.h \
    ../src/plc/pmc_g_input_scanner.h \
    ../src/plc/pmc_io_map.h \
    ../src/plc/pmc_output_handler.h \
    ../src/plc/signal_utils.h \
    ../src/task/task.h \
    ../src/threads/threads_api.h \
    ../src/threads/thread_manager.h \
    ../src/threads/rt_sem.h \
    ../src/threads/threads_config.h \
    ../src/threads/threads_controller.h \
    ../src/utils/math/cnc_math.h \
    ../src/utils/log/log.h \
    ../src/plc/plctask.h \

SOURCES += \
    ../src/compensation/comp_data.c \
    ../src/datadef/datatype.c \
    ../src/datadef/event_queue.c \
    ../src/datadef/gcode_queue.c \
    ../src/datadef/global.c \
    ../src/datadef/mcode_queue.c \
    ../src/datadef/plan_queue.c \
    ../src/datadef/rt_data.c \
    ../src/hal/hal_driver.c \
    ../src/hal/hal_io.c \
    ../src/master/master_sim.c \
    ../src/master/master.c \
    ../src/motion/auto/interpolation.c \
    ../src/motion/auto/mem.c \
    ../src/motion/auto/mem_mst.c \
    ../src/motion/auto/mem_gcode.c \
    ../src/motion/auto/parser.cpp \
    ../src/motion/auto/pathplan.c \
    ../src/motion/spindle/spl_control.c \
    ../src/motion/alarm.c \
    ../src/motion/egear_ratio.c \
    ../src/motion/emcmot.cpp \
    ../src/motion/ferror_control.c \
    ../src/motion/handwheel.c \
    ../src/motion/homing.c \
    ../src/motion/jog.c \
    ../src/motion/jog_planner.c \
    ../src/motion/stored_stroke_check.c \
    ../src/plc/pmc_a_input_scanner.c \
    ../src/plc/pmc_g_input_scanner.c \
    ../src/plc/pmc_output_handler.c \
    ../src/task/task.c \
    ../src/threads/thread_manager.cpp \
    ../src/threads/threads.cpp \
    ../src/threads/rt_sem.c \
    ../src/threads/threads_controller.cpp \
    ../src/utils/math/cnc_math.c \
    ../src/utils/log/log.c \
    ../src/plc/plctask.cpp \

INCLUDEPATH += \
    ../src/common \
    ../src/compensation \
    ../src/datadef \
    ../src/hal \
    ../src/master \
    ../src/motion \
    ../src/motion/auto \
    ../src/motion/spindle \
    ../src/monitor \
    ../src/plc \
    ../src/task \
    ../src/threads \
    ../src/utils \
    ../src/utils/math \
    ../src/utils/log \
    ../lib/kinematic/include \

# ============================================================
# 单元测试
# 统一入口：遍历 test_registry 中注册的所有测试类
SOURCES += main.cpp
HEADERS += test_registry.h

# 自动查找所有 test_* 子目录下的测试源文件（tst_*.cpp）
for(dir, $$list($$files($$PWD/test_*))) {
    !exists($$dir/*):next()           # 跳过非目录（如 test_registry.h 若命名匹配）
    files = $$files($$dir/tst_*.cpp)
    SOURCES += $$files
    INCLUDEPATH += $$dir
}


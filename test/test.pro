QT += core gui

CONFIG += qt console warn_on depend_includepath c++17
CONFIG -= app_bundle

DEFINES += MASTER_RUN_ON
DEFINES += UNIT_TEST
DEFINES += DEBUG_MODE_ENABLED

TEMPLATE = app

TARGET = tests

# 测试辅助头所在目录
INCLUDEPATH += $$PWD

# GoogleTest / GoogleMock
exists(/usr/include/gtest/gtest.h):exists(/usr/include/gmock/gmock.h) {
    message(Using system GoogleTest/GoogleMock)
    LIBS += -lgmock -lgtest
} else:exists(/usr/local/include/gtest/gtest.h):exists(/usr/local/include/gmock/gmock.h) {
    message(Using system GoogleTest/GoogleMock from /usr/local)
    INCLUDEPATH += /usr/local/include
    LIBS += -L/usr/local/lib -lgmock -lgtest
} else {
    message(Using bundled lightweight GoogleTest/GoogleMock compatibility headers)
    INCLUDEPATH += $$PWD/gtest_compat
}

# 添加共享内存库
include($$PWD/../lib/cncshm/api/apishm.pri)
LIBS += "$$PWD/../lib/cncshm/bin/libshm.a"
INCLUDEPATH += $$PWD/../lib/cncshm/api $$PWD/../lib/cncshm

# 添加译码
LIBS += -L$$PWD/../lib/parser/lib/ -lcnc_parser
INCLUDEPATH += $$PWD/../lib/parser/api $$PWD/../lib/parser

# 添加运动库
LIBS += -L"$$PWD/../lib/kinematic/bin/" -lkinematic
INCLUDEPATH += $$PWD/../lib/kinematic/include $$PWD/../lib/kinematic

# 添加误差补偿库
LIBS += -L"$$PWD/../lib/CNC_ErrorComp/lib" -lCNC_ErrorComp
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
    ../src/datadef/global.h \
    ../src/datadef/rt_data.h \
    ../src/datadef/plan_queue.h \
    ../src/datadef/rt_data.h \
    ../src/datadef/record_queue.h \
    ../src/hal/hal_driver.h \
    ../src/hal/hal_io.h \
    ../src/hal/egear_ratio.h \
    ../src/master/master_sim.h \
    ../src/master/master.h \
    ../src/motion/auto/auto.h \
    ../src/motion/auto/trajplan/auto_gcode.h \
    ../src/motion/auto/trajplan/auto_mst.h \
    ../src/motion/auto/interp/interpolation.h \
    ../src/motion/auto/pathplan/parser.h \
    ../src/motion/auto/pathplan/pathplan.h \
    ../src/motion/spindle/spl_control.h \
    ../src/motion/emcmot.h \
    ../src/motion/manual/handwheel.h \
    ../src/motion/manual/homing.h \
    ../src/motion/manual/jog/jog.h \
    ../src/motion/manual/jog/jog_planner.h \
    ../src/motion/servo_sample/servosamplesampler.h \
    ../src/motion/servo_sample/servosampleevents.h \
    ../src/motion/rerun/rerun.h \
    ../src/protect/alarm.h \
    ../src/protect/ferror_control.h \
    ../src/protect/stored_stroke_check.h \
    ../src/protect/reset.h \
    ../src/plc/lad_trace/ladtracesampler.h \
    ../src/plc/pmc_scanner/pmc_a_input_scanner.h \
    ../src/plc/pmc_scanner/pmc_g_input_scanner.h \
    ../src/plc/pmc_scanner/pmc_io_map.h \
    ../src/plc/pmc_scanner/pmc_output_handler.h \
    ../src/plc/pmc_scanner/signal_utils.h \
    ../src/plc/lad_trace/ladtracesampler.h \
    ../src/plc/plc_task/plctask.h \
    ../src/task/hmi_scanner.h \
    ../src/task/task.h \
    ../src/task/hmi_scanner.h \
    ../src/threads/threads_api.h \
    ../src/threads/thread_manager.h \
    ../src/threads/rt_sem.h \
    ../src/threads/threads_config.h \
    ../src/threads/threads_controller.h \
    ../src/utils/math/cnc_math.h \
    ../src/utils/log/log.h \
    

SOURCES += \
    ../src/compensation/comp_data.c \
    ../src/datadef/datatype.c \
    ../src/datadef/event_queue.c \
    ../src/datadef/global.c \
    ../src/datadef/plan_queue.c \
    ../src/datadef/rt_data.c \
    ../src/datadef/record_queue.c \
    ../src/hal/hal_driver.c \
    ../src/hal/hal_io.c \
    ../src/hal/egear_ratio.c \
    ../src/master/master_sim.c \
    ../src/master/master.c \
    ../src/motion/auto/auto.c \
    ../src/motion/auto/trajplan/auto_gcode.c \
    ../src/motion/auto/trajplan/auto_mst.c \
    ../src/motion/auto/interp/interpolation.c \
    ../src/motion/auto/pathplan/parser.cpp \
    ../src/motion/auto/pathplan/pathplan.c \
    ../src/motion/spindle/spl_control.c \
    ../src/motion/emcmot.cpp \
    ../src/motion/manual/handwheel.c \
    ../src/motion/manual/homing.c \
    ../src/motion/manual/jog/jog.c \
    ../src/motion/manual/jog/jog_planner.c \
    ../src/motion/servo_sample/servosamplesampler.cpp \
    ../src/motion/rerun/rerun.c \
    ../src/protect/alarm.c \
    ../src/protect/ferror_control.c \
    ../src/protect/stored_stroke_check.c \
    ../src/protect/reset.c \
    ../src/plc/lad_trace/ladtracesampler.cpp \
    ../src/plc/pmc_scanner/pmc_a_input_scanner.c \
    ../src/plc/pmc_scanner/pmc_g_input_scanner.c \
    ../src/plc/pmc_scanner/pmc_output_handler.c \
    ../src/plc/lad_trace/ladtracesampler.cpp \
    ../src/plc/plc_task/plctask.cpp \
    ../src/task/hmi_scanner.c \
    ../src/task/task.c \
    ../src/task/hmi_scanner.c \
    ../src/threads/thread_manager.cpp \
    ../src/threads/threads.cpp \
    ../src/threads/rt_sem.c \
    ../src/threads/threads_controller.cpp \
    ../src/utils/math/cnc_math.c \
    ../src/utils/log/log.c \
    

INCLUDEPATH += \
    ../src/common \
    ../src/compensation \
    ../src/datadef \
    ../src/hal \
    ../src/master \
    ../src/motion \
    ../src/motion/auto \
    ../src/motion/auto/pathplan \
    ../src/motion/auto/interp \
    ../src/motion/auto/trajplan \
    ../src/motion/manual/jog \
    ../src/motion/spindle \
    ../src/motion/manual \
    ../src/motion/servo_sample \
    ../src/motion/rerun \
    ../src/protect \
    ../src/plc/lad_trace \
    ../src/plc \
    ../src/plc/pmc_scanner \
    ../src/plc/plc_task \
    ../src/plc/lad_trace \
    ../src/task \
    ../src/threads \
    ../src/utils \
    ../src/utils/math \
    ../src/utils/log \
    ../lib/kinematic/include \

# ============================================================
# 单元测试
# 统一入口：GoogleTest 自动注册并运行所有 TEST/TEST_F
SOURCES += main.cpp test_env.cpp
HEADERS += \
    test_env.h \
    gtest_compat/gtest/gtest.h \
    gtest_compat/gmock/gmock.h

# 自动查找所有 test_* 子目录下的测试源文件（tst_*.cpp）
for(dir, $$list($$files($$PWD/test_*))) {
    !exists($$dir/*):next()
    files = $$files($$dir/tst_*.cpp)
    SOURCES += $$files
    INCLUDEPATH += $$dir
}

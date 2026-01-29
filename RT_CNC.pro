QT += core
CONFIG += c++17 cmdline

DEFINES += PROJECT_DIR=\\\"$$PWD\\\"

# ============ 编译模式配置 ============
debug_mode {
    message("========== 编译调试模式 ==========")
    DEFINES += DEBUG_MODE_ENABLED
    DEFINES += __DeveloperMode__
    CONFIG += debug
    QMAKE_CXXFLAGS += -g -O0
    QMAKE_CFLAGS += -g -O0
    TARGET = rt_cnc_d
} else {
    message("========== 编译实时模式 ==========")
    CONFIG += release
    DEFINES += MASTER_RUN_ON
    QMAKE_CXXFLAGS += -O2
    QMAKE_CFLAGS += -O2
    TARGET = rt_cnc
}

# 检查编译缓存工具 ccache 是否可用
win32 {
    if(system(ccache --version > NUL 2>&1)) {
        QMAKE_CXX = ccache $$QMAKE_CXX
        QMAKE_CC = ccache $$QMAKE_CC
    }
} else:unix {
    if(system(ccache --version > /dev/null 2>&1)) {
        QMAKE_CXX = ccache $$QMAKE_CXX
        QMAKE_CC = ccache $$QMAKE_CC
    }
}

# 检查多线程链接 lld 是否可用
unix:!macx {
    if(system(ld.lld --version > /dev/null 2>&1)) {
        QMAKE_LFLAGS += -fuse-ld=lld
    }
}

# 添加共享内存库
include(lib/cncshm/api/apishm.pri)
LIBS += -L"$$PWD/lib/cncshm/bin/" -lshm
INCLUDEPATH += $$PWD/lib/cncshm/api $$PWD/lib/cncshm

# 添加日志
include(lib/log/apilog/logapi.pri)
LIBS += -L"$$PWD/lib/log/bin/" -lcnclog
INCLUDEPATH += $$PWD/lib/log/apilog $$PWD/lib/log

# 添加译码
INCLUDEPATH += $$PWD/lib/parser/include $$PWD/lib/parser

# 添加运动库
LIBS += -L"$$PWD/lib/kinematic/bin/" -lkinematic
INCLUDEPATH += $$PWD/lib/kinematic/include $$PWD/lib/kinematic

unix:!macx: {
    DESTDIR = $$PWD/bin
}

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -Wl,--no-as-needed -lxml2 -Wl,--as-needed
LIBS += -L "/usr/local/lib/" -lethercat
INCLUDEPATH += /opt/GreeMaster/include/libGREEMASTER/


# 调试模式使用模拟主站
debug_mode {
    HEADERS += src/master/master_sim.h
    SOURCES += src/master/master_sim.c
}

# 实时模式使用真实主站
!debug_mode {
    HEADERS += src/master/main_demo.h
    SOURCES += src/master/master.c
}

HEADERS += \
    src/common/debug_mode.h \
    src/datadef/datatype.h \
    src/datadef/event_queue.h \
    src/datadef/eventdef.h \
    src/datadef/gcode_queue.h \
    src/datadef/global.h \
    src/datadef/mcode_queue.h \
    src/datadef/plan_queue.h \
    src/datadef/rt_par.h \
    src/datadef/rt_var.h \
    src/hal/hal_driver.h \
    src/hal/hal_io.h \
    src/motion/alarm.h \
    src/motion/egear_ratio.h \
    src/motion/emcmot.h \
    src/motion/ferror_control.h \
    src/motion/handwheel.h \
    src/motion/homing.h \
    src/motion/interpolation.h \
    src/motion/jog.h \
    src/motion/jog_control.h \
    src/motion/mem.h \
    src/motion/mpg.h \
    src/motion/pathplan.h \
    src/motion/rapid.h \
    src/motion/simple_tp.h \
    src/motion/stored_stroke_check.h \
    src/plc/ctimer.h \
    src/plc/cyclescanning.h \
    src/plc/hal_task.h \
    src/plc/mt_data.h \
    src/plc/nc_data.h \
    src/plc/plcdatatype.h \
    src/spindle/spindle.h \
    src/spindle/spindlestatus.h \
    src/task/pmc_input_scanner.h \
    src/task/pmc_io_map.h \
    src/task/pmc_output_handler.h \
    src/task/signal_utils.h \
    src/task/task.h \
    src/threads/threads_api.h \
    src/threads/thread_manager.h \
    src/threads/rt_sem.h \
    src/threads/threads_config.h \
    src/threads/threads_controller.h \
    src/utils/log/log.h \
    src/utils/math/cnc_math.h

SOURCES += \
    main.cpp \
    src/datadef/datatype.c \
    src/datadef/event_queue.c \
    src/datadef/gcode_queue.c \
    src/datadef/global.c \
    src/datadef/mcode_queue.c \
    src/datadef/plan_queue.c \
    src/hal/hal_driver.c \
    src/hal/hal_io.c \
    src/motion/alarm.c \
    src/motion/egear_ratio.c \
    src/motion/emcmot.cpp \
    src/motion/ferror_control.c \
    src/motion/handwheel.c \
    src/motion/homing.c \
    src/motion/interpolation.c \
    src/motion/jog.c \
    src/motion/jog_control.c \
    src/motion/mem.c \
    src/motion/mpg.c \
    src/motion/pathplan.c \
    src/motion/rapid.c \
    src/motion/simple_tp.c \
    src/motion/stored_stroke_check.c \
    src/plc/hal_task.cpp \
    src/plc/cyclescanning.cpp \
    src/spindle/spindle.c \
    src/task/pmc_input_scanner.c \
    src/task/pmc_output_handler.c \
    src/task/task.c \
    src/threads/thread_manager.cpp \
    src/threads/threads.cpp \
    src/threads/rt_sem.c \
    src/threads/threads_controller.cpp \
    src/utils/log/log.c \
    src/utils/math/cnc_math.c

INCLUDEPATH += \
    src/common \
    src/datadef \
    src/hal \
    src/master \
    src/motion \
    src/plc \
    src/task \
    src/spindle \
    src/threads \
    src/utils \
    src/utils/math \
    src/utils/log \
    lib/kinematic/include \

QT += core
CONFIG += c++17 cmdline

DEFINES += PROJECT_DIR=\\\"$$PWD\\\"

# QMAKE_CFLAGS += -fno-pie
# QMAKE_CXXFLAGS += -fno-pie
# QMAKE_LFLAGS += -no-pie

# QMAKE_CXXFLAGS += -fsanitize=address -fno-omit-frame-pointer -g
# QMAKE_CFLAGS   += -fsanitize=address -fno-omit-frame-pointer -g
# QMAKE_LFLAGS   += -fsanitize=address

QMAKE_CXXFLAGS += -fno-sized-deallocation

# ============ 编译模式配置 ============
debug_mode {
    message("========== 编译调试模式 ==========")
    DEFINES += DEBUG_MODE_ENABLED
    DEFINES += __DeveloperMode__
    CONFIG += debug
    QMAKE_CXXFLAGS += -g -O0
    QMAKE_CFLAGS += -g -O0
    TARGET = cnc_rt_d
}

release_platform_1 {
    message("========== 编译实时模式 ==========")
    message("========== 编译平台1   ==========")
    CONFIG += debug
    DEFINES += PLATFORM_1
    DEFINES += MASTER_RUN_ON
    QMAKE_CXXFLAGS += -O0
    QMAKE_CFLAGS += -O0
    TARGET = cnc_rt_1
}

release_platform_2{
    message("========== 编译实时模式 ==========")
    message("========== 编译平台2   ==========")
    CONFIG += debug
    DEFINES += MASTER_RUN_ON
    QMAKE_CXXFLAGS += -O0
    QMAKE_CFLAGS += -O0
    TARGET = cnc_rt_2
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

# 添加译码
LIBS += -L$$PWD/lib/parser/lib/ -lcnc_parser
INCLUDEPATH += $$PWD/lib/parser/api $$PWD/lib/parser

# 添加运动库
LIBS += -L"$$PWD/lib/kinematic/bin/" -lkinematic
INCLUDEPATH += $$PWD/lib/kinematic/include $$PWD/lib/kinematic

# 添加误差补偿库
LIBS += -L"$$PWD/lib/CNC_ErrorComp/lib" -lCNC_ErrorComp
INCLUDEPATH += $$PWD/lib/CNC_ErrorComp/include $$PWD/lib/CNC_ErrorComp

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
    HEADERS += src/monitor/keyboard.h
    SOURCES += src/monitor/keyboard.c
}

# 实时模式使用真实主站
!debug_mode {
    HEADERS += src/master/main_demo.h
    SOURCES += src/master/master.c
}

HEADERS += \
    src/common/debug_mode.h \
    src/compensation/comp_data.h \
    src/datadef/datatype.h \
    src/datadef/event_queue.h \
    src/datadef/eventdef.h \
    src/datadef/gcode_queue.h \
    src/datadef/global.h \
    src/datadef/rt_data.h \
    src/datadef/mcode_queue.h \
    src/datadef/plan_queue.h \
    src/datadef/rt_data.h \
    src/hal/hal_driver.h \
    src/hal/hal_io.h \
    src/motion/auto/interpolation.h \
    src/motion/auto/mem.h \
    src/motion/auto/mem_mst.h \
    src/motion/auto/mem_gcode.h \
    src/motion/auto/parser.h \
    src/motion/auto/pathplan.h \
    src/motion/spindle/spl_control.h \
    src/motion/alarm.h \
    src/motion/egear_ratio.h \
    src/motion/emcmot.h \
    src/motion/ferror_control.h \
    src/motion/handwheel.h \
    src/motion/homing.h \
    src/motion/jog/jog.h \
    src/motion/jog/jog_planner.h \
    src/motion/stored_stroke_check.h \
    src/pmc_scanner/pmc_a_input_scanner.h \
    src/pmc_scanner/pmc_g_input_scanner.h \
    src/pmc_scanner/pmc_io_map.h \
    src/pmc_scanner/pmc_output_handler.h \
    src/pmc_scanner/signal_utils.h \
    src/task/task.h \
    src/threads/threads_api.h \
    src/threads/thread_manager.h \
    src/threads/rt_sem.h \
    src/threads/threads_config.h \
    src/threads/threads_controller.h \
    src/utils/math/cnc_math.h \
    src/utils/log/log.h \
    src/plc/plctask.h \

SOURCES += \
    main.cpp \
    src/compensation/comp_data.c \
    src/datadef/datatype.c \
    src/datadef/event_queue.c \
    src/datadef/gcode_queue.c \
    src/datadef/global.c \
    src/datadef/mcode_queue.c \
    src/datadef/plan_queue.c \
    src/datadef/rt_data.c \
    src/hal/hal_driver.c \
    src/hal/hal_io.c \
    src/motion/auto/interpolation.c \
    src/motion/auto/mem.c \
    src/motion/auto/mem_mst.c \
    src/motion/auto/mem_gcode.c \
    src/motion/auto/parser.cpp \
    src/motion/auto/pathplan.c \
    src/motion/spindle/spl_control.c \
    src/motion/alarm.c \
    src/motion/egear_ratio.c \
    src/motion/emcmot.cpp \
    src/motion/ferror_control.c \
    src/motion/handwheel.c \
    src/motion/homing.c \
    src/motion/jog/jog.c \
    src/motion/jog/jog_planner.c \
    src/motion/stored_stroke_check.c \
    src/pmc_scanner/pmc_a_input_scanner.c \
    src/pmc_scanner/pmc_g_input_scanner.c \
    src/pmc_scanner/pmc_output_handler.c \
    src/task/task.c \
    src/threads/thread_manager.cpp \
    src/threads/threads.cpp \
    src/threads/rt_sem.c \
    src/threads/threads_controller.cpp \
    src/utils/math/cnc_math.c \
    src/utils/log/log.c \
    src/plc/plctask.cpp \

INCLUDEPATH += \
    src/common \
    src/compensation \
    src/datadef \
    src/hal \
    src/master \
    src/motion \
    src/motion/auto \
    src/motion/jog \
    src/motion/spindle \
    src/monitor \
    src/pmc_scanner \
    src/plc \
    src/task \
    src/threads \
    src/utils \
    src/utils/math \
    src/utils/log \
    lib/kinematic/include \

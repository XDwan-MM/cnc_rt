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

release_platform {
    message("========== 编译实时模式 ==========")
    CONFIG += debug
    DEFINES += MASTER_RUN_ON
    QMAKE_CXXFLAGS += -O0
    QMAKE_CFLAGS += -O0
    TARGET = cnc_rt
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
unix:!macx: LIBS += "$$PWD/lib/cncshm/bin/libshm.a"
else: LIBS += -L"$$PWD/lib/cncshm/bin/" -lshm
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
}

# 实时模式使用真实主站
!debug_mode {
    HEADERS += src/master/main_demo.h
    HEADERS += src/master/cJSON.h
    HEADERS += src/master/device.h
    HEADERS += src/master/slave_list.h
    HEADERS += src/master/master_wcf.h
    SOURCES += src/master/master_wcf.c
}

HEADERS += \
    src/common/debug_mode.h \
    src/compensation/comp_data.h \
    src/datadef/datatype.h \
    src/datadef/event_queue.h \
    src/datadef/eventdef.h \
    src/datadef/global.h \
    src/datadef/rt_data.h \
    src/datadef/plan_queue.h \
    src/datadef/rt_data.h \
    src/datadef/record_queue.h \
    src/hal/hal_io.h \
    src/hal/egear_ratio.h \
    src/motion/auto/auto.h \
    src/motion/auto/trace_publisher.h \
    src/motion/auto/trajplan/auto_gcode.h \
    src/motion/auto/trajplan/auto_mst.h \
    src/motion/auto/interp/interpolation.h \
    src/motion/auto/pathplan/parser.h \
    src/motion/auto/pathplan/pathplan.h \
    src/motion/auto/pathplan/hmi_request.h \
    src/motion/spindle/spl_control.h \
    src/motion/spindle/spl_orient.h \
    src/motion/spindle/spl_vel_planner.h \
    src/motion/spindle/spl_pos_planner.h \
    src/motion/emcmot.h \
    src/motion/manual/handwheel.h \
    src/motion/manual/ref.h \
    src/motion/manual/jog/jog.h \
    src/motion/manual/jog/jog_planner.h \
    src/motion/servo_sample/servosamplesampler.h \
    src/motion/servo_sample/servosampleevents.h \
    src/motion/rerun/rerun.h \
    src/motion/sysprog/tapreturn.h \
    src/motion/sysprog/sysprog.h\
    src/protect/alarm.h \
    src/protect/ferror_control.h \
    src/protect/stored_stroke_check.h \
    src/protect/reset.h \
    src/plc/plc_scanner/plc_a_signal_scanner.h \
    src/plc/plc_scanner/plc_g_signal_scanner.h \
    src/plc/plc_scanner/plc_x_signal_scanner.h \
    src/plc/plc_scanner/plc_event_types.h \
    src/plc/plc_scanner/plc_f_signal_handler.h \
    src/plc/plc_scanner/signal_utils.h \
    src/plc/plc_task/plctask.h \
    src/plc/lad_trace/ladtracesampler.h \
    src/task/task.h \
    src/task/hmi_scanner.h \
    src/threads/threads_api.h \
    src/threads/thread_manager.h \
    src/threads/rt_sem.h \
    src/threads/threads_config.h \
    src/threads/threads_controller.h \
    src/utils/math/cnc_math.h \
    src/utils/log/log.h \
    src/master/master_wcf.h \
    src/hal/driver_hal.h \
    src/hal/driver.h \
    src/hal/ethercat_driver_hal.h \
    

SOURCES += \
    main.cpp \
    src/compensation/comp_data.c \
    src/datadef/datatype.c \
    src/datadef/event_queue.c \
    src/datadef/global.c \
    src/datadef/plan_queue.c \
    src/datadef/rt_data.c \
    src/datadef/record_queue.c \
    src/hal/hal_io.c \
    src/hal/egear_ratio.c \
    src/motion/auto/auto.c \
    src/motion/auto/trace_publisher.c \
    src/motion/auto/trajplan/auto_gcode.c \
    src/motion/auto/trajplan/auto_mst.c \
    src/motion/auto/interp/interpolation.c \
    src/motion/auto/pathplan/parser.cpp \
    src/motion/auto/pathplan/pathplan.c \
    src/motion/auto/pathplan/hmi_request.c \
    src/motion/spindle/spl_control.c \
    src/motion/spindle/spl_orient.c \
    src/motion/spindle/spl_vel_planner.c \
    src/motion/spindle/spl_pos_planner.c \
    src/motion/emcmot.cpp \
    src/motion/manual/handwheel.c \
    src/motion/manual/ref.c \
    src/motion/manual/jog/jog.c \
    src/motion/manual/jog/jog_planner.c \
    src/motion/servo_sample/servosamplesampler.cpp \
    src/motion/rerun/rerun.c \
    src/motion/sysprog/tapreturn.c \
    src/motion/sysprog/sysprog.c\
    src/protect/alarm.c \
    src/protect/ferror_control.c \
    src/protect/stored_stroke_check.c \
    src/protect/reset.c \
    src/plc/plc_scanner/plc_a_signal_scanner.c \
    src/plc/plc_scanner/plc_g_signal_scanner.c \
    src/plc/plc_scanner/plc_x_signal_scanner.c \
    src/plc/plc_scanner/plc_f_signal_handler.c \
    src/plc/plc_task/plctask.cpp \
    src/plc/lad_trace/ladtracesampler.cpp \
    src/task/task.c \
    src/task/hmi_scanner.c \
    src/threads/thread_manager.cpp \
    src/threads/threads.cpp \
    src/threads/rt_sem.c \
    src/threads/threads_controller.cpp \
    src/utils/math/cnc_math.c \
    src/utils/log/log.c \
    src/hal/driver_hal.c \
    src/hal/driver.c \
    src/hal/ethercat_driver_hal.c \


INCLUDEPATH += \
    src/common \
    src/compensation \
    src/datadef \
    src/hal \
    src/master \
    src/motion \
    src/motion/auto \
    src/motion/auto/pathplan \
    src/motion/auto/interp \
    src/motion/auto/trajplan \
    src/motion/manual/jog \
    src/motion/spindle \
    src/motion/manual \
    src/motion/servo_sample \
    src/motion/rerun \
    src/motion/sysprog \
    src/protect \
    src/plc \
    src/plc/plc_scanner \
    src/plc/plc_task \
    src/plc/lad_trace \
    src/task \
    src/threads \
    src/utils \
    src/utils/math \
    src/utils/log \
    lib/kinematic/include \

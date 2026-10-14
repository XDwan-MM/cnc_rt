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

# 曾经有个 offline_ledger 档：用 -D 打开两个"现场确认开关"，另编一个
# bin/cnc_rt_ledger_d 给离线主轴回归用（台账不开时运动类请求会被适配层整条丢掉）。
# 那两个开关已删（见 src/hal/rt_hal_config.c 顶部注释）：台账现在只反映"参数域
# 给全了没有"，握手端把四轴参数写全台账就清了 —— 所以离线回归直接用 bin/cnc_rt_d。

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

# 添加nurbs库
 LIBS += -L"$$PWD/lib/nurbs/bin/" -lnurbsfit
 INCLUDEPATH += $$PWD/lib/nurbs/include $$PWD/lib/nurbsfit

# 添加误差补偿库
LIBS += -L"$$PWD/lib/CNC_ErrorComp/lib" -lCNC_ErrorComp
INCLUDEPATH += $$PWD/lib/CNC_ErrorComp/include $$PWD/lib/CNC_ErrorComp

# 添加圆弧过渡库
LIBS += -L"$$PWD/lib/circleSmooth/bin" -lcircleSmooth
INCLUDEPATH += $$PWD/lib/circleSmooth/include $$PWD/lib/circleSmooth

unix:!macx: {
    DESTDIR = $$PWD/bin
}

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -Wl,--no-as-needed -lxml2 -Wl,--as-needed -lm
INCLUDEPATH += /opt/GreeMaster/include/libGREEMASTER/

# ============ HAL 接入 ============
# 原来的 -lethercat（libethercat.so，旧驱动层）已移除：那一层连同 src/master/master.c
# 一起被 HAL 取代。两者不能共存——HAL 静态库与 libethercat.so 有 31 个重名符号
# （ethercat_init / ethercat_close / g_device_data 等），同一次链接里谁在前谁赢，
# 另一个的调用会绑到签名不同的版本上，实测其中一种顺序直接段错误。
#
# 依赖随工程走：与 lib/parser、lib/kinematic、lib/CNC_ErrorComp 同一约定——
# HAL 的头与静态库作为 vendored 副本放在 lib/cnc_hal/ 下，不引用外部路径、
# 不依赖环境变量。HAL 改了之后跑 lib/cnc_hal/sync.sh 重新同步（来源记在 SYNCED.txt）。
LIBS += -L"$$PWD/lib/cnc_hal/lib" -lcnc_hal
INCLUDEPATH += $$PWD/lib/cnc_hal/include


# 调试模式：驱动层用内存桩，不接硬件也能跑 RT 逻辑。
# 注意这不是"另一套 RT 逻辑"——适配层和周期循环两种模式共用，
# 差别只在这一份驱动层实现。
debug_mode {
    SOURCES += src/hal/rt_hal_sim_backend.c
    # 桩必须扮演 HAL 的驱动层，所以要看得到 HAL 的内部类型
    # （DeviceSlot / DevDictRole / Ds402Request）。**只有调试构建**加这条路径，
    # 实时构建仍然只见公共头，依赖边界不被破坏。
    INCLUDEPATH += $$PWD/lib/cnc_hal/internal
}

# 实时模式：驱动层来自 HAL 静态库。
# src/master/ 整个目录与 src/hal/ 的旧驱动层（driver/driver_hal/ethercat_driver_hal/
# hal_io/egear_ratio）已随接入完成删除——它们全被 HAL 取代，保留只会让人以为
# 还有两条路径。需要时从基线提交 4222562 捞回。

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
    src/motion/auto/auto.h \
    src/motion/auto/trace_publisher.h \
    src/motion/auto/trajplan/auto_gcode.h \
    src/motion/auto/trajplan/auto_mst.h \
    src/motion/auto/interp/interpolation.h \
    src/motion/auto/pathplan/parser.h \
    src/motion/auto/pathplan/pathplan.h \
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
    src/utils/rt_axis_view.h \
    src/utils/log/log.h \
    lib/nurbs/include/nurbsfit.h \
    lib/circleSmooth/include/circle_smooth.h \
    

SOURCES += \
    main.cpp \
    src/compensation/comp_data.c \
    src/datadef/datatype.c \
    src/datadef/event_queue.c \
    src/datadef/global.c \
    src/datadef/plan_queue.c \
    src/datadef/rt_data.c \
    src/datadef/record_queue.c \
    src/hal/rt_hal_config.c \
    src/hal/rt_hal_adapter.c \
    src/motion/auto/auto.c \
    src/motion/auto/trace_publisher.c \
    src/motion/auto/trajplan/auto_gcode.c \
    src/motion/auto/trajplan/auto_mst.c \
    src/motion/auto/interp/interpolation.c \
    src/motion/auto/pathplan/parser.cpp \
    src/motion/auto/pathplan/pathplan.c \
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
    src/utils/rt_axis_view.c \
    src/utils/log/log.c \
    #lib/nurbs/nurbsfit.c \
    #lib/circleSmooth/circle_smooth.c \


INCLUDEPATH += \
    src/common \
    src/compensation \
    src/datadef \
    src/hal \
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
    lib/nurbs/include \
    lib/circleSmooth/include \


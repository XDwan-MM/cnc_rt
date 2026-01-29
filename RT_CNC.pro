QT += core
CONFIG += c++17 cmdline

# 使用 CONFIG+=debug_mode 启用调试模式编译
# 使用 CONFIG+=release 或不加参数使用实时模式编译
# 调试模式编译: qmake RT_CNC.pro CONFIG+=debug_mode && make
# 实时模式编译: qmake RT_CNC.pro && make

debug_mode {
    message("========== 编译调试模式 ==========")
    DEFINES += DEBUG_MODE_ENABLED
    DEFINES += __DeveloperMode__
    QMAKE_CXXFLAGS += -g
    QMAKE_CFLAGS += -g
} else {
    message("========== 编译实时模式 ==========")
    QMAKE_CXXFLAGS += -O2
    QMAKE_CFLAGS += -O2
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

# 添加日志
include(lib/log/apilog/logapi.pri)
LIBS += -L"$$PWD/lib/log/bin/" -lcnclog
INCLUDEPATH += $$PWD/lib/log/apilog $$PWD/lib/log

TARGET = RT_CNC

unix:!macx: {
    DESTDIR = $$PWD/bin
}

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -Wl,--no-as-needed -lxml2 -Wl,--as-needed

LIBS += -L "/usr/local/lib/" -lethercat

INCLUDEPATH += \
    /opt/GreeMaster/include/libGREEMASTER/

HEADERS += \
    src/common/debug_mode.h \
    src/common/greecnc.h \
    src/common/rt_sem.h \
    src/common/threads_controller.h \
    src/hal/hal_driver.h \
    src/hal/hal_io.h \
    src/master/main_demo.h \
    src/master/master.h \
    src/master/master_sim.h \
    src/threads/api/threads_api.h \
    src/threads/src/thread_manager.h

SOURCES += \
    main.cpp \
    src/common/rt_sem.c \
    src/common/threads_controller.cpp \
    src/hal/hal_driver.c \
    src/hal/hal_io.c \
    src/master/master.c \
    src/master/master_sim.c \
    src/threads/src/thread_manager.cpp \
    src/threads/src/threads.cpp

INCLUDEPATH += \
    src/hal \
    src/common \
    src/master \
    src/threads/api \
    src/threads/src \

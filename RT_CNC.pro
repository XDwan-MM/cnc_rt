QT += core
CONFIG += c++17 cmdline

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

CONFIG += debug

TARGET = RT_CNC

unix:!macx: {
    DESTDIR = $$PWD/bin
}

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -lxml2

LIBS += -L "/usr/local/lib/" -lethercat

INCLUDEPATH += \

HEADERS += \
    src/common/greecnc.h \
    src/common/rt_sem.h \
    src/hal/hal_driver.h \
    src/hal/hal_io.h \
    src/master/main_demo.h \
    src/master/master.h

SOURCES += \
    main.cpp \
    src/common/rt_sem.c \
    src/hal/hal_driver.c \
    src/hal/hal_io.c \
    src/master/master.c

INCLUDEPATH += \
    src/hal \
    src/common \
    src/master \

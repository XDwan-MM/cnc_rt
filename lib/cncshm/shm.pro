CONFIG += c++17

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

# SHM 库的构建项目
TEMPLATE = lib
CONFIG += shared 

QT += core

DEFINES += SHM_LIBRARY

unix {
    target.path = /usr/local/lib
}
!isEmpty(target.path): INSTALLS += target

TARGET = shm
DESTDIR = $$PWD/bin

SOURCES += \
    private/alarmapi.cpp \
    private/alarmmanager.cpp \
    private/hmisysapi.cpp \
    private/hmisysmanager.cpp \
    private/parapi.cpp \
    private/parmanager.cpp \
    private/shmmanager.cpp \
    private/regmanager.cpp \
    private/ladmanager.cpp \
    private/shmapi.cpp \
    private/regapi.cpp \
    private/ladapi.cpp \
    private/varapi.cpp \
    private/varmanager.cpp

HEADERS += \
    private/alarmmanager.h \
    private/hmisysmanager.h \
    private/ncdata.h \
    private/parmanager.h \
    private/regmanager.h \
    private/semaphorelocker.h \
    private/ladmanager.h  \
    private/shmmanager.h \
    private/vardata.h \
    private/varmanager.h

include(./api/apishm.pri)


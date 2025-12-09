CONFIG += c++17
if(system(ccache > NUL 2>&1)) {
    QMAKE_CXX        = ccache $$QMAKE_CXX
    QMAKE_CC         = ccache $$QMAKE_CC
    QMAKE_LINK       = ccahce $$QMAKE_LINK
    QMAKE_LINK_SHLIB = ccache $$QMAKE_LINK_SHLIB
}
# DAL 库的构建项目
TEMPLATE = lib
CONFIG += shared 

QT += core

DEFINES += DAL_LIBRARY

TARGET = dal
DESTDIR = ../../../lib

SOURCES += \
    shmmanager.cpp \
    regmanager.cpp \
    ladmanager.cpp \
    shmapi.cpp \
    regapi.cpp \
    ladapi.cpp

HEADERS += \
    ncdata.h \
    regmanager.h \
    semaphorelocker.h \
    ladmanager.h 

include(../api/apidal.pri)


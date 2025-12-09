CONFIG += c++17
if(system(ccache > NUL 2>&1)) {
    QMAKE_CXX        = ccache $$QMAKE_CXX
    QMAKE_CC         = ccache $$QMAKE_CC
    QMAKE_LINK       = ccahce $$QMAKE_LINK
    QMAKE_LINK_SHLIB = ccache $$QMAKE_LINK_SHLIB
}

include(dal/api/apidal.pri)
include(bll/bll.pri)
include(views/views.pri)

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = plc
DESTDIR = ../bin

SOURCES += \
    main.cpp

HEADERS += \

LIBS += -L../lib -ldal

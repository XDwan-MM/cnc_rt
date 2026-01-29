QT -= gui

TEMPLATE = lib
DEFINES += CNCLOGMAIN_LIBRARY

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    logapi.cpp \
    logentry.cpp \
    logmanager.cpp \
    spdlogmanager.cpp

HEADERS += \
#    ../apilog/logapi.h \
#    ../apilog/logentry.h \
#    ../apilog/logmanager.h \
#    ../apilog/spdlogmanager.h \
    cnclog_global.h

include (../apilog/logapi.pri)
INCLUDEPATH += $$PWD/../apilog/ $$PWD/spdlog/

# Default rules for deployment.
unix {
    target.path = /usr/local/lib
}

DESTDIR = $$PWD/../bin

!isEmpty(target.path): INSTALLS += target

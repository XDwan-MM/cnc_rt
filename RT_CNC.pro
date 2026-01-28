QT += core
CONFIG += c++17 cmdline

CONFIG += debug

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -lxml2


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

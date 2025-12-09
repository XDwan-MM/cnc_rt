QT += core
CONFIG += c++17 cmdline

CONFIG += debug

LIBS += -L/opt/GreeMaster/lib -lGREEMASTER -lpthread -lxml2

QMAKE_CFLAGS += -fno-pie
QMAKE_CXXFLAGS += -fno-pie
QMAKE_LFLAGS += -no-pie


# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000 # disables all the APIs deprecated before Qt 6.0.0

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += \
    ./Threads \
    ./common

HEADERS += \
    alarm/alarm_manager.h \
    common/command_type.h \
    common/greecnc.h \
    common/rt_sem.h \
    common/threads_controller.h \
    datadef/common.h \
    datadef/gcode_queue.h \
    datadef/plan_queue.h \
    datadef/shm.h \
    hal/hal_driver.h \
    kinematics/trajectory.h \
    master/master.h \
    master/src/master_maco.h \
    master/src/master_var.h \
    math/cnc_math.h \
    motion/emcmot.h \
    motion/emcmotcfg.h \
    motion/handwheel.h \
    motion/homing.h \
    motion/interpolation.h \
    motion/mem.h \
    motion/mot_priv.h \
    motion/motion.h \
    motion/motion_struct.h \
    motion/pathplan.h \
    motion/usrmotintf.h \
    parser/src/text_buffer.h \
    task/api/taskapi.h \
    task/api/taskdataapi.h \
    task/core/emc_type.h \
    task/core/emccfg.h \
    task/core/emcglb.h \
    task/core/kevent.h \
    task/core/shcommand.h \
    task/src/emctaskeventscan.h \
    task/src/task.h \
    parser/include/ParserState.h \
    parser/include/abstracttextbuffer.h \
    parser/include/coordinate.h \
    parser/include/itpparam.h \
    parser/include/motiondata.h \
    parser/include/ncinterpreter.h \
    parser/include/ncitperrortype.h \
    parser/src/ci_pmc.h \
    task/core/CircularBuffer.h \
    task/core/ThreadSafeSharedData.h \
    task/core/cmd_msg.h \
    task/core/status_msg.h \
    task/core/threadsafeshareddata_c.h \
    task/src/taskintf.h \
    test/cnc_shm.h \
    test/shm_write.h \
    threads/api/threads_api.h \
    threads/src/thread_manager.h \
    utils/cJSON.h \
    utils/cJSON_Utils.h \
    utils/json_read.h \
    utils/json_write.h

SOURCES += \
    common/rt_sem.c \
    common/threads_controller.c \
    datadef/common.c \
    datadef/gcode_queue.c \
    datadef/plan_queue.c \
    datadef/shm.c \
    hal/hal_driver.c \
    main.c \
    master/master.c \
    math/cnc_math.c \
    motion/emcmot.c \
    motion/handwheel.c \
    motion/homing.c \
    motion/interpolation.c \
    motion/mem.c \
    motion/motion.c \
    motion/pathplan.c \
    motion/usrmotintf.cpp \
    parser/src/text_buffer.cpp \
    task/core/cmd_msg.cpp \
    task/core/emcglb.cpp \
    task/core/kevent.cpp \
    task/core/shcommand.cpp \
    task/src/emctask.cpp \
    task/src/emctaskeventscan.c \
    task/core/threadsafeshareddata_c.cpp \
    task/src/emctaskmain.cpp \
    task/src/task.cpp \
    task/src/taskintf.cpp \
    test/shm_write.c \
    threads/src/thread_manager.c \
    threads/src/threads.c \
    utils/cJSON.c \
    utils/cJSON_Utils.c \
    utils/json_read.c \
    utils/json_write.c


INCLUDEPATH += \
    common \
    alarm \
    parser/include \
    parser/src \
    task/api \
    task/core \
    task/src \
    threads/api \
    threads/src \
    threads/core \
    PLC/src/dal/api \
    hal \
    motion \
    master\
    /opt/GreeMaster/include/libGREEMASTER/ \
    datadef/ \
    test/ \

unix:!macx: LIBS += -L$$PWD/lib/ -lcnc_parser

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: PRE_TARGETDEPS += $$PWD/lib/libcnc_parser.a


unix:!macx: LIBS += -L$$PWD/lib/ -lkinematics

INCLUDEPATH += $$PWD/''
DEPENDPATH += $$PWD/''

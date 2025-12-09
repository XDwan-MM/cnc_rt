CONFIG += c++17
if(system(ccache > NUL 2>&1)) {
    QMAKE_CXX        = ccache $$QMAKE_CXX
    QMAKE_CC         = ccache $$QMAKE_CC
    QMAKE_LINK       = ccahce $$QMAKE_LINK
    QMAKE_LINK_SHLIB = ccache $$QMAKE_LINK_SHLIB
}

# 引入dal动态库
include(../api/apidal.pri)

# 指定项目类型为应用程序
TEMPLATE = app

# 将 Qt 测试库添加到项目中，以便使用 Qt 的测试功能。
QT += core testlib
# 从项目中移除 GUI 相关模块，因为这是一个控制台应用程序，不需要图形界面。
QT -= gui

# qt：确保项目使用 Qt 框架。
# console：将应用程序设置为控制台程序。
# warn_on：在编译时启用警告信息。
# depend_includepath：将所有依赖项的 INCLUDEPATH 添加到项目中。
# testcase：将项目设置为一个测试用例。
CONFIG += qt console warn_on depend_includepath testcase

# 取消应用程序捆绑包设置，避免在 macOS 上创建一个 .app 捆绑包
CONFIG -= app_bundle

TARGET = tests

SOURCES += \
    main.cpp \
    testshm.cpp \
    testreg.cpp \
    testlad.cpp

HEADERS += \
    testshm.h \
    testreg.h \
    testlad.h

LIBS += -L../../../lib -ldal

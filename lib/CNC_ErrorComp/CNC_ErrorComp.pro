TEMPLATE  = lib
CONFIG += staticlib
CONFIG -= app_bundle
CONFIG -= qt

# # 强制使用 C 编译标准 (针对 C 文件)
# QMAKE_CFLAGS += -std=c11

TARGET = CNC_ErrorComp

# 指定公共头文件包含路径
INCLUDEPATH += $$PWD/include

# 核心头文件
HEADERS += \
    include/comp_common.h \
    include/rt_par.h \
    include/pitch_comp.h \
    include/perp_comp.h

# 源代码实现
SOURCES += \
    src/pitch_comp.c \
    src/pitch_comp_table_init.c \
    src/perp_comp.c

# 构建输出路径配置(隔离)
DESTDIR     = $$PWD/lib
# OBJECTS_DIR = $$PWD/build

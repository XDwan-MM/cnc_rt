TEMPLATE = subdirs 

CONFIG += c++17

# 添加你的子项目
SUBDIRS += \
    libdal \
    tests

# 设置子项目的路径
libdal.file = ./src/libdal.pro
tests.subdir = ./tests

# 设置子项目的依赖
tests.depends = libdal

CONFIG += ordered

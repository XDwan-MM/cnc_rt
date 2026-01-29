CONFIG += c++17

TEMPLATE = subdirs 

# 添加子项目
SUBDIRS += \
    cnclog \
    tests

# 设置子项目的路径
cnclog.file = ./src/cnclog.pro
tests.file = ./tests/tests.pro

# 设置子项目的依赖
tests.depends = cnclog

# 依照顺序条件编译
CONFIG += ordered

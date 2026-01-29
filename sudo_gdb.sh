#!/bin/sh
# 使用 sudo 启动 gdb，并传递所有参数 ("$@")
exec sudo /usr/bin/gdb "$@"
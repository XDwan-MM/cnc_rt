### 1、创建cpp

添加新文件夹：test_*

添加单元测试：tst_*


### 2、修改test.pro文件

如果 CNC_RT.pro 增加头文件和源文件，请同步更新 test.pro


### 3、运行 build.sh

sudo ./build.sh clean 清除可执行文件

sudo ./build.sh 生成可执行文件

sudo ./build.sh run 运行可执行文件

```SHELL
sudo -E LD_LIBRARY_PATH=../lib/cncshm/bin ./bin/tests
********* Start testing of TestMath *********
Config: Using QtTest library 5.15.13, Qt 5.15.13 (x86_64-little_endian-lp64 shared (dynamic) release build; by GCC 13.2.0), ubuntu 24.04
PASS   : TestMath::initTestCase()
PASS   : TestMath::test_is_equal()
PASS   : TestMath::cleanupTestCase()
Totals: 6 passed, 0 failed, 0 skipped, 0 blacklisted, 1ms
********* Finished testing of TestMath *********
********* Start testing of TestUtils *********
PASS   : TestUtils::initTestCase()
PASS   : TestUtils::test_string()
PASS   : TestUtils::cleanupTestCase()
Totals: 4 passed, 0 failed, 0 skipped, 0 blacklisted, 0ms
********* Finished testing of TestUtils *********
```
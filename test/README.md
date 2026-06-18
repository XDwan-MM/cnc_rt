### 0、安装gtest

```BASH
sudo apt update
sudo apt install libgtest-dev libgmock-dev cmake
```

### 1、创建cpp

添加新文件夹：test_*

添加单元测试：tst_*

测试文件使用 GoogleTest / GoogleMock：

```cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>

class TestDemo : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(TestDemo, Example)
{
    EXPECT_EQ(1 + 1, 2);
}
```

### 2、修改test.pro文件

如果 CNC_RT.pro 增加头文件和源文件，请同步更新 test.pro

如果系统已安装 GoogleTest/GoogleMock，test.pro 会优先链接系统库；未安装时使用 test/gtest_compat 下的轻量兼容头，支持当前测试使用的 TEST/TEST_F、EXPECT/ASSERT、GTEST_SKIP、MOCK_METHOD、EXPECT_CALL(...).Times(n)。


### 3、运行 build.sh

sudo ./build.sh clean 清除可执行文件

sudo ./build.sh 生成可执行文件

sudo ./build.sh run 运行可执行文件

```SHELL
sudo ./bin/tests
[==========] Running 35 tests.
[ RUN      ] TestMath.IsEqual
[       OK ] TestMath.IsEqual
[ RUN      ] TestAuto.AutoExecuteG00ArriveCheck
[       OK ] TestAuto.AutoExecuteG00ArriveCheck
[==========] 35 tests ran.
[  PASSED  ] 35 tests.
```

/********************************************************************
 * @file        main.cpp
 * @brief       单元测试统一入口
 *              遍历 test_registry 中注册的全部测试类并依次运行，
 *              任一测试失败则整体返回非 0。
********************************************************************/

#include <QtTest>
#include "test_registry.h"

int main(int argc, char *argv[])
{
    int status = 0;
    for (const TestFactory &factory : testRegistry()) {
        QObject *test = factory();
        // 共享同一组命令行参数（如 -v2），各测试独立累计返回码
        status |= QTest::qExec(test, argc, argv);
        delete test;
    }
    return status;
}

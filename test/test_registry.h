#ifndef TEST_REGISTRY_H
#define TEST_REGISTRY_H

/********************************************************************
 * @file        test_registry.h
 * @brief       QtTest 测试自动注册机制
 *              各子目录下的 tst_*.cpp 通过 REGISTER_TEST 注册测试类，
 *              由统一的 main.cpp 遍历运行，从而合并进单个可执行文件。
********************************************************************/

#include <QObject>
#include <QList>
#include <functional>

// 测试类工厂：返回一个新建的测试对象
typedef std::function<QObject *()> TestFactory;

// 全局测试注册表（函数内静态变量，避免跨翻译单元初始化顺序问题）
inline QList<TestFactory> &testRegistry()
{
    static QList<TestFactory> registry;
    return registry;
}

// 注册器：构造时把工厂登记到全局表
struct TestRegistrar {
    explicit TestRegistrar(TestFactory factory)
    {
        testRegistry().append(factory);
    }
};

// 在 tst_*.cpp 中调用，注册一个测试类
#define REGISTER_TEST(ClassName)                                  \
    static TestRegistrar reg_##ClassName(                         \
        []() -> QObject * { return new ClassName(); });

#endif // TEST_REGISTRY_H

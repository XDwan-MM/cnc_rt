#ifndef TEST_GTEST_COMPAT_GTEST_H_
#define TEST_GTEST_COMPAT_GTEST_H_

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace testing {

class Test;

namespace internal {

struct TestInfo {
    std::string suite_name;
    std::string test_name;
    std::function<Test*()> factory;
};

inline std::vector<TestInfo>& TestRegistry()
{
    static std::vector<TestInfo> tests;
    return tests;
}

class FatalFailure : public std::exception {
public:
    explicit FatalFailure(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }

private:
    std::string message_;
};

class SkipFailure : public std::exception {
public:
    explicit SkipFailure(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }

private:
    std::string message_;
};

inline int& CurrentFailureCount()
{
    static int count = 0;
    return count;
}

inline std::vector<std::function<void()>>& PerTestTearDownHooks()
{
    static std::vector<std::function<void()>> hooks;
    return hooks;
}

inline void AddPerTestTearDownHook(std::function<void()> hook)
{
    PerTestTearDownHooks().push_back(std::move(hook));
}

inline std::string MakeLocation(const char* file, int line)
{
    std::ostringstream oss;
    oss << file << ":" << line;
    return oss.str();
}

template <typename T>
std::string ValueToString(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

inline std::string ValueToString(bool value)
{
    return value ? "true" : "false";
}

inline std::string ValueToString(std::nullptr_t)
{
    return "nullptr";
}

template <typename T>
std::string ValueToString(T* value)
{
    std::ostringstream oss;
    oss << static_cast<const void*>(value);
    return oss.str();
}

inline void AddFailure(const char* file, int line, const std::string& message, bool fatal)
{
    ++CurrentFailureCount();
    std::cerr << MakeLocation(file, line) << ": Failure\n" << message << std::endl;
    if (fatal) {
        throw FatalFailure(message);
    }
}

inline void ExpectTrue(bool condition, const char* expr, const char* file, int line, bool fatal)
{
    if (!condition) {
        AddFailure(file, line, std::string("Value of: ") + expr + "\n  Actual: false\nExpected: true", fatal);
    }
}

inline void ExpectFalse(bool condition, const char* expr, const char* file, int line, bool fatal)
{
    if (condition) {
        AddFailure(file, line, std::string("Value of: ") + expr + "\n  Actual: true\nExpected: false", fatal);
    }
}

template <typename A, typename B>
void ExpectEq(const A& actual, const B& expected, const char* actual_expr, const char* expected_expr,
              const char* file, int line, bool fatal)
{
    if (!(actual == expected)) {
        std::ostringstream oss;
        oss << "Expected equality of these values:\n  " << actual_expr << "\n    Which is: "
            << ValueToString(actual) << "\n  " << expected_expr << "\n    Which is: "
            << ValueToString(expected);
        AddFailure(file, line, oss.str(), fatal);
    }
}

template <typename A, typename B>
void ExpectNe(const A& actual, const B& expected, const char* actual_expr, const char* expected_expr,
              const char* file, int line, bool fatal)
{
    if (!(actual != expected)) {
        std::ostringstream oss;
        oss << "Expected inequality of these values:\n  " << actual_expr << "\n  " << expected_expr
            << "\nBoth are: " << ValueToString(actual);
        AddFailure(file, line, oss.str(), fatal);
    }
}

inline bool DoublesAlmostEqual(double a, double b)
{
    if (a == b) {
        return true;
    }
    const double diff = std::fabs(a - b);
    const double scale = std::max(std::fabs(a), std::fabs(b));
    return diff <= 4.0 * std::numeric_limits<double>::epsilon() * scale;
}

template <typename A, typename B>
void ExpectDoubleEq(const A& actual, const B& expected, const char* actual_expr,
                    const char* expected_expr, const char* file, int line, bool fatal)
{
    const double a = static_cast<double>(actual);
    const double e = static_cast<double>(expected);
    if (!DoublesAlmostEqual(a, e)) {
        std::ostringstream oss;
        oss << "Expected equality of these values:\n  " << actual_expr << "\n    Which is: " << a
            << "\n  " << expected_expr << "\n    Which is: " << e;
        AddFailure(file, line, oss.str(), fatal);
    }
}

template <typename A, typename B, typename C>
void ExpectNear(const A& actual, const B& expected, const C& abs_error, const char* actual_expr,
                const char* expected_expr, const char* error_expr, const char* file, int line,
                bool fatal)
{
    const double a = static_cast<double>(actual);
    const double e = static_cast<double>(expected);
    const double tol = static_cast<double>(abs_error);
    if (!(std::fabs(a - e) <= tol)) {
        std::ostringstream oss;
        oss << "The difference between " << actual_expr << " and " << expected_expr << " is "
            << (a - e) << ", which exceeds " << error_expr << " (" << tol << ")";
        AddFailure(file, line, oss.str(), fatal);
    }
}

struct LtCmp { template <typename A, typename B> bool operator()(const A& a, const B& b) const { return a < b; } };
struct LeCmp { template <typename A, typename B> bool operator()(const A& a, const B& b) const { return a <= b; } };
struct GtCmp { template <typename A, typename B> bool operator()(const A& a, const B& b) const { return a > b; } };
struct GeCmp { template <typename A, typename B> bool operator()(const A& a, const B& b) const { return a >= b; } };

template <typename A, typename B, typename Cmp>
void ExpectCmp(const A& actual, const B& expected, const char* actual_expr,
               const char* expected_expr, const char* op, const char* file, int line, bool fatal)
{
    if (!Cmp()(actual, expected)) {
        std::ostringstream oss;
        oss << "Expected: (" << actual_expr << ") " << op << " (" << expected_expr << ")\n"
            << "  Actual: " << ValueToString(actual) << " vs " << ValueToString(expected);
        AddFailure(file, line, oss.str(), fatal);
    }
}

class SkipStream {
public:
    SkipStream(const char* file, int line)
    {
        stream_ << MakeLocation(file, line) << ": Skipped";
    }

    ~SkipStream() noexcept(false)
    {
        throw SkipFailure(stream_.str());
    }

    template <typename T>
    SkipStream& operator<<(const T& value)
    {
        stream_ << "\n" << value;
        return *this;
    }

private:
    std::ostringstream stream_;
};

using TestFactory = std::function<Test*()>;

inline bool RegisterTest(const char* suite_name, const char* test_name, TestFactory factory)
{
    TestRegistry().push_back(TestInfo{suite_name, test_name, std::move(factory)});
    return true;
}

} // namespace internal

class Test {
public:
    virtual ~Test() = default;

protected:
    virtual void SetUp() {}
    virtual void TearDown() {}

private:
    virtual void TestBody() = 0;
    friend class UnitTest;
};

class Environment {
public:
    virtual ~Environment() = default;
    virtual void SetUp() {}
    virtual void TearDown() {}
};

inline std::vector<Environment*>& Environments()
{
    static std::vector<Environment*> environments;
    return environments;
}

inline Environment* AddGlobalTestEnvironment(Environment* environment)
{
    Environments().push_back(environment);
    return environment;
}

inline void InitGoogleTest(int*, char**) {}

class UnitTest {
public:
    static UnitTest* GetInstance()
    {
        static UnitTest instance;
        return &instance;
    }

    int Run()
    {
        for (Environment* environment : Environments()) {
            environment->SetUp();
        }

        int failed_tests = 0;
        int skipped_tests = 0;
        const auto& tests = internal::TestRegistry();

        std::cout << "[==========] Running " << tests.size() << " tests." << std::endl;
        for (const internal::TestInfo& info : tests) {
            std::cout << "[ RUN      ] " << info.suite_name << "." << info.test_name << std::endl;
            internal::CurrentFailureCount() = 0;
            bool skipped = false;
            std::unique_ptr<Test> test(info.factory());

            try {
                test->SetUp();
                test->TestBody();
                for (const auto& hook : internal::PerTestTearDownHooks()) {
                    hook();
                }
                test->TearDown();
            } catch (const internal::SkipFailure& skip) {
                skipped = true;
                std::cout << skip.what() << std::endl;
                try {
                    for (const auto& hook : internal::PerTestTearDownHooks()) {
                        hook();
                    }
                    test->TearDown();
                } catch (...) {
                    internal::AddFailure(__FILE__, __LINE__, "TearDown threw after skip", true);
                }
            } catch (const internal::FatalFailure&) {
                try {
                    for (const auto& hook : internal::PerTestTearDownHooks()) {
                        hook();
                    }
                    test->TearDown();
                } catch (...) {
                    internal::AddFailure(__FILE__, __LINE__, "TearDown threw after fatal failure", true);
                }
            } catch (const std::exception& ex) {
                internal::AddFailure(__FILE__, __LINE__, std::string("Unhandled exception: ") + ex.what(), false);
            } catch (...) {
                internal::AddFailure(__FILE__, __LINE__, "Unhandled non-standard exception", false);
            }

            if (skipped) {
                ++skipped_tests;
                std::cout << "[  SKIPPED ] " << info.suite_name << "." << info.test_name << std::endl;
            } else if (internal::CurrentFailureCount() == 0) {
                std::cout << "[       OK ] " << info.suite_name << "." << info.test_name << std::endl;
            } else {
                ++failed_tests;
                std::cout << "[  FAILED  ] " << info.suite_name << "." << info.test_name << std::endl;
            }
        }

        for (auto it = Environments().rbegin(); it != Environments().rend(); ++it) {
            (*it)->TearDown();
        }

        std::cout << "[==========] " << tests.size() << " tests ran." << std::endl;
        if (skipped_tests > 0) {
            std::cout << "[  SKIPPED ] " << skipped_tests << " tests." << std::endl;
        }
        if (failed_tests > 0) {
            std::cout << "[  FAILED  ] " << failed_tests << " tests." << std::endl;
            return 1;
        }
        std::cout << "[  PASSED  ] " << (tests.size() - skipped_tests) << " tests." << std::endl;
        return 0;
    }
};

} // namespace testing

#define TEST_F(FixtureName, TestName)                                                               \
    class FixtureName##_##TestName##_Test : public FixtureName {                                    \
    private:                                                                                        \
        void TestBody() override;                                                                   \
        static const bool registered_;                                                              \
    };                                                                                              \
    const bool FixtureName##_##TestName##_Test::registered_ =                                       \
        ::testing::internal::RegisterTest(#FixtureName, #TestName,                                  \
            []() -> ::testing::Test* { return new FixtureName##_##TestName##_Test(); });            \
    void FixtureName##_##TestName##_Test::TestBody()

#define TEST(SuiteName, TestName)                                                                   \
    class SuiteName##_##TestName##_Test : public ::testing::Test {                                  \
    private:                                                                                        \
        void TestBody() override;                                                                   \
        static const bool registered_;                                                              \
    };                                                                                              \
    const bool SuiteName##_##TestName##_Test::registered_ =                                         \
        ::testing::internal::RegisterTest(#SuiteName, #TestName,                                    \
            []() -> ::testing::Test* { return new SuiteName##_##TestName##_Test(); });              \
    void SuiteName##_##TestName##_Test::TestBody()

#define EXPECT_TRUE(condition)                                                                      \
    ::testing::internal::ExpectTrue(static_cast<bool>(condition), #condition, __FILE__, __LINE__, false)
#define ASSERT_TRUE(condition)                                                                      \
    ::testing::internal::ExpectTrue(static_cast<bool>(condition), #condition, __FILE__, __LINE__, true)
#define EXPECT_FALSE(condition)                                                                     \
    ::testing::internal::ExpectFalse(static_cast<bool>(condition), #condition, __FILE__, __LINE__, false)
#define ASSERT_FALSE(condition)                                                                     \
    ::testing::internal::ExpectFalse(static_cast<bool>(condition), #condition, __FILE__, __LINE__, true)
#define EXPECT_EQ(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectEq(gtest_actual_value, gtest_expected_value, #actual, #expected,  \
                                      __FILE__, __LINE__, false);                                    \
    } while (0)
#define ASSERT_EQ(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectEq(gtest_actual_value, gtest_expected_value, #actual, #expected,  \
                                      __FILE__, __LINE__, true);                                     \
    } while (0)
#define EXPECT_NE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectNe(gtest_actual_value, gtest_expected_value, #actual, #expected,  \
                                      __FILE__, __LINE__, false);                                    \
    } while (0)
#define ASSERT_NE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectNe(gtest_actual_value, gtest_expected_value, #actual, #expected,  \
                                      __FILE__, __LINE__, true);                                     \
    } while (0)
#define EXPECT_DOUBLE_EQ(actual, expected)                                                          \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectDoubleEq(gtest_actual_value, gtest_expected_value, #actual,       \
                                            #expected, __FILE__, __LINE__, false);                   \
    } while (0)
#define ASSERT_DOUBLE_EQ(actual, expected)                                                          \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectDoubleEq(gtest_actual_value, gtest_expected_value, #actual,       \
                                            #expected, __FILE__, __LINE__, true);                    \
    } while (0)
#define EXPECT_NEAR(actual, expected, abs_error)                                                    \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        const auto& gtest_abs_error_value = (abs_error);                                             \
        ::testing::internal::ExpectNear(gtest_actual_value, gtest_expected_value,                    \
                                        gtest_abs_error_value, #actual, #expected, #abs_error,       \
                                        __FILE__, __LINE__, false);                                  \
    } while (0)
#define ASSERT_NEAR(actual, expected, abs_error)                                                    \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        const auto& gtest_abs_error_value = (abs_error);                                             \
        ::testing::internal::ExpectNear(gtest_actual_value, gtest_expected_value,                    \
                                        gtest_abs_error_value, #actual, #expected, #abs_error,       \
                                        __FILE__, __LINE__, true);                                   \
    } while (0)
#define EXPECT_LT(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::LtCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, "<",                       \
            __FILE__, __LINE__, false);                                                              \
    } while (0)
#define ASSERT_LT(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::LtCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, "<",                       \
            __FILE__, __LINE__, true);                                                               \
    } while (0)
#define EXPECT_LE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::LeCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, "<=",                      \
            __FILE__, __LINE__, false);                                                              \
    } while (0)
#define ASSERT_LE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::LeCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, "<=",                      \
            __FILE__, __LINE__, true);                                                               \
    } while (0)
#define EXPECT_GT(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::GtCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, ">",                       \
            __FILE__, __LINE__, false);                                                              \
    } while (0)
#define ASSERT_GT(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::GtCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, ">",                       \
            __FILE__, __LINE__, true);                                                               \
    } while (0)
#define EXPECT_GE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::GeCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, ">=",                      \
            __FILE__, __LINE__, false);                                                              \
    } while (0)
#define ASSERT_GE(actual, expected)                                                                 \
    do {                                                                                            \
        const auto& gtest_actual_value = (actual);                                                   \
        const auto& gtest_expected_value = (expected);                                               \
        ::testing::internal::ExpectCmp<decltype(gtest_actual_value),                                 \
            decltype(gtest_expected_value), ::testing::internal::GeCmp>(                             \
            gtest_actual_value, gtest_expected_value, #actual, #expected, ">=",                      \
            __FILE__, __LINE__, true);                                                               \
    } while (0)
#define GTEST_SKIP() ::testing::internal::SkipStream(__FILE__, __LINE__)
#define RUN_ALL_TESTS() ::testing::UnitTest::GetInstance()->Run()

#endif // TEST_GTEST_COMPAT_GTEST_H_

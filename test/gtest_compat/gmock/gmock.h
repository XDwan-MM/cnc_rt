#ifndef TEST_GTEST_COMPAT_GMOCK_H_
#define TEST_GTEST_COMPAT_GMOCK_H_

#include <gtest/gtest.h>

#include <map>
#include <sstream>
#include <string>

namespace testing {
namespace internal {

struct MockExpectation {
    int expected_times = 0;
    int actual_times = 0;
};

inline std::string NormalizeMockMethod(const std::string& call)
{
    const std::string::size_type paren = call.find('(');
    std::string method = paren == std::string::npos ? call : call.substr(0, paren);
    while (!method.empty() && method.back() == ' ') {
        method.pop_back();
    }
    const std::string::size_type last_space = method.find_last_of(" \t");
    if (last_space != std::string::npos) {
        method = method.substr(last_space + 1);
    }
    return method;
}

inline std::map<std::string, MockExpectation>& MockRegistry()
{
    static std::map<std::string, MockExpectation> registry;
    return registry;
}

inline std::string MockKey(const void* object, const std::string& method)
{
    std::ostringstream oss;
    oss << object << "#" << method;
    return oss.str();
}

class ExpectationBuilder {
public:
    ExpectationBuilder(const void* object, const char* call)
        : object_(object), method_(NormalizeMockMethod(call))
    {
    }

    void Times(int expected_times) const
    {
        MockRegistry()[MockKey(object_, method_)].expected_times = expected_times;
    }

private:
    const void* object_;
    std::string method_;
};

inline void RecordMockCall(const void* object, const char* method)
{
    MockRegistry()[MockKey(object, method)].actual_times++;
}

inline void VerifyAndClearMockExpectations()
{
    for (const auto& item : MockRegistry()) {
        const MockExpectation& expectation = item.second;
        if (expectation.actual_times != expectation.expected_times) {
            std::ostringstream oss;
            oss << "Expected mock call count " << expectation.expected_times
                << ", actual " << expectation.actual_times << " for " << item.first;
            AddFailure(__FILE__, __LINE__, oss.str(), false);
        }
    }
    MockRegistry().clear();
}

struct MockHookRegistration {
    MockHookRegistration()
    {
        AddPerTestTearDownHook(VerifyAndClearMockExpectations);
    }
};

inline MockHookRegistration& MockHook()
{
    static MockHookRegistration registration;
    return registration;
}

} // namespace internal

class Mock {
public:
    template <typename T>
    static bool VerifyAndClearExpectations(T*)
    {
        internal::VerifyAndClearMockExpectations();
        return true;
    }
};

} // namespace testing

#define MOCK_METHOD(return_type, name, args, specs) \
    return_type name args { ::testing::internal::RecordMockCall(this, #name); }

#define EXPECT_CALL(mock_object, call) \
    (::testing::internal::MockHook(), ::testing::internal::ExpectationBuilder(&(mock_object), #call))

#endif // TEST_GTEST_COMPAT_GMOCK_H_

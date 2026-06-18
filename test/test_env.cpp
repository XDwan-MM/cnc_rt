#include "test_env.h"

static bool g_test_env_ready = false;
static std::string g_test_env_message = "测试环境未初始化";

void testEnvSetReady(bool ready, const std::string &message)
{
    g_test_env_ready = ready;
    g_test_env_message = message;
}

bool testEnvReady()
{
    return g_test_env_ready;
}

const std::string &testEnvMessage()
{
    return g_test_env_message;
}

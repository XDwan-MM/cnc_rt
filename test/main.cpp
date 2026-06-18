/********************************************************************
 * @file        main.cpp
 * @brief       GoogleTest 单元测试统一入口
********************************************************************/

#include <gtest/gtest.h>

#include "test_env.h"

#include "datatype.h"
#include "global.h"
#include <shmapi.h>

namespace {

class RtTestEnvironment : public ::testing::Environment {
public:
    void SetUp() override
    {
        if (!shmInit()) {
            testEnvSetReady(false, "共享内存未就绪");
            return;
        }

        GNC_RES res = rt_global_init();
        if (res == GNC_OPERATOR_SUCCESS) {
            testEnvSetReady(true);
        } else {
            testEnvSetReady(false, "rt_global_init 失败(" + std::to_string(res) + ")");
        }
    }

    void TearDown() override
    {
        rt_global_delete();
    }
};

} // namespace

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new RtTestEnvironment);
    return RUN_ALL_TESTS();
}

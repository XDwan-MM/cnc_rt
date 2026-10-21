#include <stdio.h>
#include <stdlib.h>

#include "hal_c_api.h"
#include "rt_hal_config.h"

int main(void) {
    const int gree4 = getenv("CNC_RT_SIM_GREE4") && atoi(getenv("CNC_RT_SIM_GREE4"));
    const int expected_slave[RT_HAL_AXIS_COUNT] = {gree4 ? 2 : 3, gree4 ? 2 : 4,
                                                   gree4 ? 2 : 5, 2};
    const int expected_axis[RT_HAL_AXIS_COUNT] = {gree4 ? 1 : 0, gree4 ? 2 : 0,
                                                  gree4 ? 3 : 0, 0};
    HalCConfig cfg;
    HalContext* context = NULL;
    char err[192];
    if (rt_hal_config_build_and_check(&cfg, err, sizeof(err)) != HAL_OK ||
        hal_context_create(&cfg, &context, err, sizeof(err)) != HAL_OK ||
        hal_context_start(context, err, sizeof(err)) != HAL_OK) {
        fprintf(stderr, "启动失败: %s\n", err);
        hal_context_destroy(context);
        return 1;
    }
    if (hal_rt_wait_cycle(context) != HAL_OK ||
        hal_rt_begin_cycle(context) != HAL_OK) {
        fprintf(stderr, "采样失败\n");
        hal_context_destroy(context);
        return 1;
    }
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        HalCCapability capability;
        if (hal_axis_capability(context, i, HAL_FUNC_POSITION, &capability) != HAL_OK ||
            capability.state != HAL_CAP_READY ||
            capability.slave_pos != expected_slave[i] ||
            capability.axis_index != expected_axis[i]) {
            fprintf(stderr, "轴 %d 绑定错误\n", i);
            hal_context_destroy(context);
            return 1;
        }
    }
    if (hal_rt_commit_cycle(context) != HAL_OK) {
        fprintf(stderr, "提交失败\n");
        hal_context_destroy(context);
        return 1;
    }
    hal_context_destroy(context);
    printf("%s 轴绑定通过\n", gree4 ? "四合一" : "六从站");
    return 0;
}

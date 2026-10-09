#include "master_wcf.h"

#include "main_demo.h"
#include "rt_sem.h"

#include "global.h"

#include <alarmapi.h>
#include <hmisysapi.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "device.h"
#include "log.h"

#include "driver.h"
#include "driver_hal.h"

/* =========================================================
 * Developer Debug
 * ========================================================= */

#ifdef __DeveloperMode__

#define debug printf

#else

#define debug(...)

#endif

/* =========================================================
 * 全局变量
 * ========================================================= */

int master_error_code = 0;

/*
 * 驱动器故障复位请求标志：
 * true  表示 Master_Run 需要发送 6040=0x0080；
 * 待反馈确认 6041 Fault 位和 error_code 清零后恢复为 false。
 */
bool reset_sem = false;

master_context_t g_master;

/*
 * 驱动器名称
 *
 * 0 : 主轴
 * 1 : X
 * 2 : Y
 * 3 : Z
 */
static const char *drv_names[] = {"主轴", "X", "Y", "Z"};

/* =========================================================
 * 内部函数
 * ========================================================= */

static void Master_Error_Internal(int code);

/* =========================================================
 * Master_Start
 * ========================================================= */

int Master_Start(void)
{
    int ret;

    op_flag = 0;

    /* -----------------------------------------------------
     * 初始化 Master 上下文
     * ----------------------------------------------------- */

    memset(&g_master, 0, sizeof(g_master));

    g_master.state = MASTER_STATE_INIT;

    master_error_code = 0;

    /* -----------------------------------------------------
     * 1. 初始化 EtherCAT Master
     * ----------------------------------------------------- */

    master_error_code = ethercat_init();

    if (master_error_code < 0)
    {
        Master_Error_Internal(master_error_code);

        g_master.error_code = master_error_code;

        g_master.state = MASTER_STATE_ERROR;

        return MASTER_ERR_INIT;
    }

    /* ethercat_init() 完成后 g_device_data 才有效。 */
    int result = 0;
    result = Hal_Driver_Array_Init();
    if (result != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "驱动器数组初始化失败 %d", result);
        g_master.state = MASTER_STATE_ERROR;
        return MASTER_ERR_INIT;
    }

    // 清一下驱动器报错
    Driver_Fault_Reset(&x_axis_driver);
    Driver_Fault_Reset(&y_axis_driver);
    Driver_Fault_Reset(&z_axis_driver);
    Driver_Fault_Reset(&spindle_driver);

    /* 进给轴使用 CSP；主轴使用 CSV，但主轴首次启动保持下使能。 */
    Driver_Request_Mode(&x_axis_driver, DRV_MODE_CSP);
    Driver_Request_Mode(&y_axis_driver, DRV_MODE_CSP);
    Driver_Request_Mode(&z_axis_driver, DRV_MODE_CSP);
    Driver_Request_Mode(&spindle_driver, DRV_MODE_CSV);
    Driver_Disable(&spindle_driver);

    g_master.initialized = true;

    g_master.state = MASTER_STATE_STARTED;

    /* -----------------------------------------------------
     * 2. 检查 EtherCAT 硬件状态
     * ----------------------------------------------------- */

    Master_Hardware_Error();

    /* -----------------------------------------------------
     * 3. 初始 PDO 接收
     *
     * 注意：
     *
     * 这里执行一次：
     *
     * GM_Master_Receive()
     *      +
     * Hal_Driver_Update_All(true, false)
     *
     * Driver层不会再次读取。
     * ----------------------------------------------------- */

    ret = Master_Receive();

    if (ret < 0)
    {
        LOG(LOG_LEVEL_ERR, "EtherCAT启动阶段PDO接收失败");

        g_master.state = MASTER_STATE_ERROR;

        return ret;
    }

    /* 首次使能前锁定位置目标，避免 CSP 从默认值跳到当前位置。 */
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_axis_driver.actual_pos);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_axis_driver.actual_pos);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_axis_driver.actual_pos);
    Hal_Driver_Set_Target_Vel(&spindle_driver, 0);

    /*
     * 首次使能进给轴前，先用实际反馈位置锁定目标位置，
     * 防止 CSP 上使能时产生位置跳变。主轴不在启动阶段使能，
     * 仍保持 Switch On Disabled，等待 STOP/CW/CCW 事件。
     */
    Driver_Enable(&x_axis_driver);
    Driver_Enable(&y_axis_driver);
    Driver_Enable(&z_axis_driver);

    /* 让 emcmot 首个周期用实际反馈重建指令位置，避免首次 JOG 产生位置跳变。 */
    update_pos_flag = 0;


    /* -----------------------------------------------------
     * 4. IO资源扫描
     * ----------------------------------------------------- */

    if (IO_Resource_Allocation_Scanf() != 0)
    {
        LOG(LOG_LEVEL_ERR, "驱动打开失败");
    }

    /* -----------------------------------------------------
     * 5. 启动前 PDO 写
     * ----------------------------------------------------- */

    debug("==============================握手前DMA写================================\n");

    ret = GM_Master_Send(1);

    if (ret < 0)
    {
        LOG(LOG_LEVEL_ERR, "EtherCAT启动阶段PDO发送失败，错误码 %d", ret);

        g_master.error_code = ret;

        g_master.state = MASTER_STATE_ERROR;

        return MASTER_ERR_SEND;
    }

    /* -----------------------------------------------------
     * 6. Master OP Valid
     * ----------------------------------------------------- */

    debug("====================与主站握手:GM_Master_OP_Valid()=======================\n");

    ret = GM_Master_OP_Valid();

    if (ret < 0)
    {
        LOG(LOG_LEVEL_ERR, "GM_Master_OP_Valid失败，错误码 %d", ret);

        g_master.error_code = ret;

        g_master.state = MASTER_STATE_ERROR;

        return MASTER_ERR_INIT;
    }

    /* -----------------------------------------------------
     * 7. Master正式进入OP
     * ----------------------------------------------------- */

    g_master.op_valid = true;

    g_master.state = MASTER_STATE_OP;

    /* -----------------------------------------------------
     * 8. 通知其他实时线程
     * ----------------------------------------------------- */

    sem_post(&op_sem);

    LOG(LOG_LEVEL_INFO, "EtherCAT Master进入OP状态");

    return MASTER_SUCCESS;
}

/* =========================================================
 * Master_Close
 * ========================================================= */

int Master_Close(void)
{
    int ret;

    op_flag = 0;

    if (!g_master.initialized)
    {
        g_master.state = MASTER_STATE_CLOSED;

        return MASTER_SUCCESS;
    }

    ret = ethercat_close();

    if (ret < 0)
    {
        RT_LOG(ERROR, "关闭EtherCAT失败，错误码 %d", ret);

        LOG(LOG_LEVEL_ERR, "关闭EtherCAT失败，错误码 %d", ret);

        g_master.error_code = ret;

        g_master.state = MASTER_STATE_ERROR;

        return MASTER_ERR_CLOSE;
    }

    g_master.initialized = false;

    g_master.op_valid = false;

    g_master.state = MASTER_STATE_CLOSED;

    return MASTER_SUCCESS;
}

/* =========================================================
 * Master_Receive
 *
 * EtherCAT反馈PDO唯一读取入口
 * ========================================================= */
int Master_Receive(void)
{
    int ret;

    /* -----------------------------------------------------
     * 1. EtherCAT接收
     *
     * 从站PDO → Master Process Image
     * ----------------------------------------------------- */

    debug("=============================接收主站周期数据==============================\n");

    ret = GM_Master_Receive(1, &PdoWarnInfo);

    if (ret < 0)
    {
        LOG(LOG_LEVEL_ERR, "GM_Master_Receive失败，错误码 %d", ret);

        g_master.error_code = ret;

        return MASTER_ERR_RECEIVE;
    }

    /* -----------------------------------------------------
     * 2. PDO通信警告
     * ----------------------------------------------------- */

    if (PdoWarnInfo.PDOWarnFlag || PdoWarnInfo.DCWarnFlag)
    {
        PdoWar_Info_Get(NULL, &PdoWarnInfo);
    }

    /* -----------------------------------------------------
     * 3. CRC / Timeout统计
     * ----------------------------------------------------- */

    if (PdoWarnInfo.FlameTimeOutCount || PdoWarnInfo.CRCErrCount)
    {
        /*
         * 后续可以在这里统一处理
         * EtherCAT通信异常。
         */
    }

    /* -----------------------------------------------------
     * 4. 统一读取所有驱动器反馈
     *
     * 这是整个系统唯一的反馈PDO读取位置。
     *
     * 每个周期只执行一次。
     *
     * 读取的数据包括：
     *
     * 6041 Statusword
     * 6064 Position Actual Value
     * 606C Velocity Actual Value
     * 6077 Torque Actual Value
     * Error Code
     * 6061 Mode Display
     *
     * 读取之后全部缓存到：
     *
     * hal_driver_t
     *
     * 后续 Driver 层只读取缓存。
     * ----------------------------------------------------- */

    ret = Hal_Driver_Update_All(true, false);

    if (ret != HAL_DRIVER_SUCCESS)
    {
        printf("(ret != HAL_DRIVER_SUCCESS) err %d\n", ret);
        LOG(LOG_LEVEL_ERR, "Hal_Driver_Update_All(true, false)失败，错误码 %d", ret);

        g_master.error_code = ret;

        return MASTER_ERR_RECEIVE;
    }

    return MASTER_SUCCESS;
}

/* =========================================================
 * Master_Send
 * ========================================================= */

int Master_Send(void)
{
    int ret;

    /* -----------------------------------------------------
     * 1. Driver HAL缓存 → EtherCAT PDO
     * ----------------------------------------------------- */

    ret = Hal_Driver_Update_All(false, true);

    if (ret != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "Hal_Driver_Update_All(write)失败，错误码 %d", ret);

        g_master.error_code = ret;

        return MASTER_ERR_SEND;
    }

    /* -----------------------------------------------------
     * 2. EtherCAT PDO发送
     * ----------------------------------------------------- */

    debug("=============================发送主站周期数据==============================\n");

    ret = GM_Master_Send(1);

    if (ret < 0)
    {
        LOG(LOG_LEVEL_ERR, "GM_Master_Send失败，错误码 %d", ret);

        g_master.error_code = ret;

        return MASTER_ERR_SEND;
    }

    return MASTER_SUCCESS;
}

/* =========================================================
 * Master_Wait_Sync
 * ========================================================= */

void Master_Wait_Sync(void)
{
#ifndef MASTER_RUN_ON

    /*
     * 非实时模式
     */

    op_flag = 1;

    post_rt_sem_all();

    usleep(1000);

#else
    /*
     * 实时模式
     */

    int ret;

    /* -----------------------------------------------------
     * 1. 等待EtherCAT周期
     * ----------------------------------------------------- */

    debug("==============================等待周期中断================================\n");

    ret = GM_Wait_Master_Sync(IRQ_TIMEOUT);

    if (ret < 0)
    {
        op_flag = 0;

        g_master.error_code = ret;

        LOG(LOG_LEVEL_ERR, "GM_Wait_Master_Sync失败，错误码 %d", ret);

        return;
    }

    /* -----------------------------------------------------
     * 2. 接收反馈PDO
     *
     * 注意：
     *
     * 这里调用 Master_Receive()
     *
     * Master_Receive() 是唯一反馈读取入口。
     * ----------------------------------------------------- */

    ret = Master_Receive();

    if (ret < 0)
    {
        op_flag = 0;

        return;
    }

    /* -----------------------------------------------------
     * 3. 当前周期开始
     * ----------------------------------------------------- */

    op_flag = 1;

    /* -----------------------------------------------------
     * 4. 唤醒其他实时线程
     * ----------------------------------------------------- */

    sem_post(&plc_sem);

    sem_post(&motion_sem);

    sem_post(&path_plan_sem);

#endif
}

/* =========================================================
 * Master_Run
 * ========================================================= */

void Master_Run(void)
{
    int ret;

    while (!thread_exit_flag)
    {
        /* -------------------------------------------------
         * ① 检查EtherCAT硬件状态
         * ------------------------------------------------- */

        Master_Hardware_Error();

        /* -------------------------------------------------
         * ② 等待周期
         *
         * 内部已经完成：
         *
         * Wait Sync
         *     ↓
         * Receive PDO
         *     ↓
         * 更新 Driver HAL反馈
         *     ↓
         * 唤醒 PLC / Motion / Path
         * ------------------------------------------------- */

        Master_Wait_Sync();

        if (op_flag == 0)
        {
            continue;
        }

        /* -------------------------------------------------
         * ③ 等待运动控制线程
         * ------------------------------------------------- */

        if (sem_wait(&moton_sem_exec_done) != 0)
        {
            if (thread_exit_flag)
            {
                break;
            }

            continue;
        }

        /* -------------------------------------------------
         * ④ Fault Reset请求
         * ------------------------------------------------- */

        if (reset_sem)
        {
            /*
             * Driver层负责实际的CiA402 Fault Reset。
             *
             * 注意：
             *
             * 这里不能直接读取/写入 EtherCAT PDO。
             */

            ret = Driver_Fault_Reset_All();

            if (ret == HAL_DRIVER_SUCCESS)
            {
                // printf("所有驱动器控制字都写了0x0080\n");
                bool all_reset = true;

                for (uint8_t i = 0; i < dri_manager.count; ++i)
                {
                    hal_driver_t *drv = dri_manager.drivers[i];
                    bool fault_active;
                    bool error_active;

                    if (drv == NULL)
                    {
                        continue;
                    }

                    /* status_word/error_code 来自本周期 Master_Receive() 的缓存。 */
                    fault_active = ((drv->status_word & 0x0008U) != 0);
                    error_active = (drv->error_code != 0);

                    if (fault_active || error_active)
                    {
                        all_reset = false;
                        continue;
                    }

                    if (drv->state == DRV_STATE_FAULT)
                    {
                        drv->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;
                        drv->act_mode = DRV_MODE_NONE;
                        drv->mode_switch_state = DRV_MODE_SWITCH_IDLE;
                        drv->mode_switch_target = DRV_MODE_NONE;
                        drv->mode_switch_source = DRV_MODE_NONE;
                        drv->mode_switch_result = HAL_DRIVER_SUCCESS;
                        drv->mode_switch_cycles = 0;
                    }

                    if (i == 0)
                    {
                        rt_data->axis_data[3].status.is_error = false;
                        delAlarm(ALARM_AX, ALARM_ERROR, 33);
                    }
                    else if (i <= 3)
                    {
                        rt_data->axis_data[i - 1].status.is_error = false;
                        delAlarm(ALARM_AX, ALARM_ERROR, 29 + i);
                    }

                }

                /* 等待下一周期反馈确认后再结束复位流程。 */
                if (all_reset)
                {
                    reset_sem = false;
                }
            }
        }

        /* -------------------------------------------------
         * ⑤ Driver CiA402状态机
         *
         * 这里不再读取 EtherCAT PDO。
         *
         * Driver_Cyclic_Process()
         * 直接使用：
         *
         * drv->status_word
         * drv->actual_pos
         * drv->actual_vel
         * drv->actual_torque
         * drv->error_code
         * drv->act_mode
         *
         * ------------------------------------------------- */

        ret = Driver_Cyclic_Process_All();

        if (ret != HAL_DRIVER_SUCCESS)
        {
            LOG(LOG_LEVEL_ERR, "Driver_Cyclic_Process_All失败，错误码 %d", ret);
        }

        /* -------------------------------------------------
         * ⑥ 检查驱动器报警
         * ------------------------------------------------- */

        Check_Driver_Error();

        /* -------------------------------------------------
         * ⑦ 发送PDO
         * ------------------------------------------------- */

        ret = Master_Send();

        if (ret < 0)
        {
            op_flag = 0;

            continue;
        }

        /* -------------------------------------------------
         * ⑧ 周期计数
         * ------------------------------------------------- */

        g_master.cycle_count++;
    }
}

/* =========================================================
 * Master_Reset
 * ========================================================= */

void Master_Reset(void)
{
    /*
     * 这里只产生Fault Reset请求。
     *
     * 不直接访问 EtherCAT PDO。
     */

    reset_sem = true;
}

/* =========================================================
 * Check_Driver_Error
 * ========================================================= */

void Check_Driver_Error(void)
{
    /* Fault Reset 尚未完成时，不能用旧反馈再次建立报警状态。 */
    if (reset_sem)
    {
        return;
    }

    /* error_code/status_word 已由 Master_Receive() 读取并缓存。 */
    for (int axis = 0; axis < 4; ++axis)
    {
        /* rt_data: X/Y/Z/主轴；Driver: 主轴/X/Y/Z。 */
        const int driver_index = (axis == 3) ? 0 : axis + 1;
        hal_driver_t *drv = dri_manager.drivers[driver_index];
        bool is_error;
        bool has_fault;
        bool is_disabled;
        int alarm_no;

        if (drv == NULL)
        {
            continue;
        }

        is_error = (drv->error_code != 0);
        has_fault = ((drv->status_word & 0x0008U) != 0);
        is_disabled = rt_data->axis_data[axis].status.is_disable;
        alarm_no = (axis == 3) ? 33 : 29 + axis + 1;

        rt_data->axis_data[axis].status.is_error = is_error;

        if (is_error || has_fault)
        {
            drv->state = DRV_STATE_FAULT;

            if (is_error && !is_disabled)
            {
                int ret = setAlarm(ALARM_AX, ALARM_ERROR, alarm_no);

                if (ret != -8)
                {
                    LOG(LOG_LEVEL_ERR, "%s轴驱动器报错，错误码 %u", drv_names[driver_index], drv->error_code);
                }
            }
        }
        else
        {
            if (!has_fault && drv->state == DRV_STATE_FAULT)
            {
                drv->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;
                drv->act_mode = DRV_MODE_NONE;
            }

            delAlarm(ALARM_AX, ALARM_ERROR, alarm_no);
        }
    }
}

/* =========================================================
 * Master_Hardware_Error
 * ========================================================= */

void Master_Hardware_Error(void)
{
    int res = 0;

    switch (g_current_error_ctx.ErrorID)
    {
    case ECAT_INIT_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 100);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站初始化模块错误");

        break;

    case ECAT_CONTROL_PANEL_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 101);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "控制面板断连");

        break;

    case ECAT_IO_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 102);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "IO模块断连");

        break;

    case ECAT_SLAVE3_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 103);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "从站三断连");

        break;

    case ECAT_SLAVE4_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 104);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "从站四断连");

        break;

    case ECAT_SLAVE5_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 105);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "从站五断连");

        break;

    case ECAT_SLAVE6_CONNECT_LOST:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 106);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "从站六断连");

        break;

    case ECAT_TOPOLOGY_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 107);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站拓扑模块错误");

        break;

    case ECAT_ACTIVE_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 108);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站激活模块错误");

        break;

    case ECAT_DC_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 109);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站DC模块错误");

        break;

    case ECAT_PDO_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 110);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站PDO模块错误");

        break;

    case ECAT_RT_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 111);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "主站RT模块错误");

        break;

    case UNKOWN_ERROR:

        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 999);

        if (res == -8)
            return;

        LOG(LOG_LEVEL_ERR, "未知错误");

        break;

    default:

        break;
    }
}

/* =========================================================
 * Master_Error_Internal
 * ========================================================= */

static void Master_Error_Internal(int code)
{
    int res = 0;

    switch (code)
    {
    case -1000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 0);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "文件或设备打开失败");
        break;

    case -1001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 1);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "没有找到目标");
        break;

    case -1002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 2);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "空指针");
        break;

    case -1003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 3);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "存储空间不足");
        break;

    case -1004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 4);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "无效的参数");
        break;

    case -1005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 5);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "系统参数错误");
        break;

    case -1006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 6);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "驱动打开失败");
        break;

    case -1007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 7);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "内存分配失败");
        break;

    case -1008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 8);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "自旋锁分配失败");
        break;

    case -1009:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 9);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "传输驱动初始化失败");
        break;

    case -1010:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 10);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "信号量初始化失败");
        break;

    case -1011:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 11);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "超时");
        break;

    case -1012:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 12);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "主站无回复");
        break;

    case -1013:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 13);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "主站状态错误");
        break;

    case -2000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 20);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "无效的通信地址");
        break;

    case -2001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 21);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "内存映射失败");
        break;

    case -2002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 22);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "内存回读失败");
        break;

    case -3000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 30);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "eni文件解析错误");
        break;

    case -3001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 31);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "传入的指针已有指向地址");
        break;

    case -3002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 32);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件中没有从站");
        break;

    case -3003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 33);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有厂商号");
        break;

    case -3004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 34);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有产品号");
        break;

    case -3005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 35);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有看门狗");
        break;

    case -3006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 36);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有syncyc");
        break;

    case -3007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 37);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有syncshift");
        break;

    case -3008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 38);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站的某个SM没有方向说明");
        break;

    case -4000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 40);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "entry的长度无效");
        break;

    case -4001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 41);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "文件或设备打开失败");
        break;

    case -4002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 42);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "存在未完成的reg请求");
        break;

    case -4003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 43);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "没有已发送的reg请求");
        break;

    case -4004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 44);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "REG队列已满");
        break;

    case -4005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 45);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "存在未完成的SDO请求");
        break;

    case -4006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 46);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "没有已发送的reg请求");
        break;

    case -4007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 47);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "SDObuffer的空间不足");
        break;

    case -4008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 48);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "远程升级时计算次数与实际写入不符");
        break;

    case -9999:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 999);
        if (res == -8)
            return;
        LOG(LOG_LEVEL_ERR, "未知错误");
        break;

    default:
        break;
    }
}

/* =========================================================
 * Master_Get_State
 * ========================================================= */

master_state_t Master_Get_State(void) { return g_master.state; }

/* =========================================================
 * Master_Is_OP
 * ========================================================= */

bool Master_Is_OP(void) { return g_master.op_valid && g_master.state == MASTER_STATE_OP; }

/* =========================================================
 * Master_Error
 * ========================================================= */

int Master_Error(void) { return g_master.error_code; }

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

    /*
     * CSP进给轴上使能前先保持当前反馈位置，避免启动时把目标位置
     * 当作零点而引起轴移动。正常运动周期随后会继续更新这些目标值。
     */
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_axis_driver.actual_pos);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_axis_driver.actual_pos);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_axis_driver.actual_pos);

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
        LOG(LOG_LEVEL_ERR, "Hal_Driver_Update_All(read)失败，错误码 %d", ret);

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
     * 非实时测试模式
     */

    op_flag = 1;

    post_rt_sem_all();

    usleep(1000);

#else

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
                reset_sem = false;
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

#ifndef DEBUG_MODE_ENABLED

        Check_Driver_Error();

#endif

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
    int i;

    /*
     * Fault Reset期间暂不进行故障同步。
     */

    if (reset_sem)
    {
        return;
    }

    /*
     * rt_data轴编号：
     *
     * 0 = X
     * 1 = Y
     * 2 = Z
     * 3 = 主轴
     *
     * Driver编号：
     *
     * 0 = 主轴
     * 1 = X
     * 2 = Y
     * 3 = Z
     */

    for (i = 0; i < 4; i++)
    {
        if (rt_data->axis_data[i].status.is_error)
        {
            if (i == 3)
            {
                /* 主轴 */

                if (dri_manager.drivers[0] != NULL)
                {
                    dri_manager.drivers[0]->state = DRV_STATE_FAULT;
                }
            }
            else
            {
                /* X / Y / Z */

                if (dri_manager.drivers[i + 1] != NULL)
                {
                    dri_manager.drivers[i + 1]->state = DRV_STATE_FAULT;
                }
            }
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

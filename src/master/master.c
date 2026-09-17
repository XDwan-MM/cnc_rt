#include "master.h"
#include "main_demo.h"
#include "rt_sem.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <stdio.h>
#include <unistd.h>
#include "log.h"
#include "device.h"
#include "spl_control.h"

#ifdef __DeveloperMode__
#define debug printf
#else
#define debug(...)
#endif

// ===============错误检查宏=================
#define CHECK_RC(rc,info,label) \
do { \
        if ((rc) < 0) { \
            printf("%s,rc = %d\n", info,rc); \
            goto label; \
    } \
} while(0)
#define CHECK_FUNC(func) do { \
    int ret = (func); \
    if (ret < 0) { \
        fprintf(stderr, "Error in function: %s, rc = %d\n", #func, ret); \
} \
} while (0)

// 分析理解：master表示主站(代表CNC控制器) slave表示从站(代表轴设备-驱动器)，还有其他设备 面板 IO模组
// 输出给驱动器
// 6040 控制字 16位
// 6060 操作模式 8位
// 607A 目标位置 32位

// 驱动器返回给CNC
// 6041 状态字 16位
// 6061 当前驱动器的模式 8位
// 6064 实际位置 32位
// 606C 实际速度 32位

/**
 * @brief 设备编号枚举
 */
typedef enum {
    DEV_ID_CONTROL_PANEL  = 0,  // 0: 控制面板设备
    DEV_ID_IO_MODULE      = 1,  // 1: IO模块设备
    DEV_ID_SPINDLE_DRIVER = 2,  // 2: 主轴驱动器设备
    DEV_ID_X_AXIS_DRIVER  = 3,  // 3: X轴驱动器设备
    DEV_ID_Y_AXIS_DRIVER  = 4,  // 4: Y轴驱动器设备
    DEV_ID_Z_AXIS_DRIVER  = 5,  // 5: Z轴驱动器设备

    DEV_ID_COUNT                // 6: 设备总数（用于数组大小定义及越界检查）
} DeviceId_e;

// 全局变量定义
int master_error_code = 0;
bool reset_sem = false;
static char* drv_names[] = {"主","X","Y","Z"};

static void Master_Receive();
static void Master_Error(int code);
static void Master_Hardware_Error(ErrorContext g_current_error_ctx);

int Master_Start() {
    master_error_code = ethercat_init();
    if ( master_error_code < 0) {
        Master_Error(master_error_code);
        return -1;
    }
    Master_Hardware_Error(g_current_error_ctx);
    Master_Receive();
    return 1;
}

int Master_Close() {
    int error_code = ethercat_close();
    if ( error_code < 0) {
        RT_LOG(ERROR,  "关闭EtherCAT失败，错误码 %d", error_code);
        LOG(LOG_LEVEL_ERR, "关闭EtherCAT失败，错误码 %d", error_code);
        return -1;
    }
}

// 等待中断信号
void Master_Wait_Sync(){
    #ifndef MASTER_RUN_ON
        post_rt_sem_all();
        op_flag = 1;
        usleep(1000);
    #else
        debug("==============================等待周期中断================================\n");
        rc = GM_Wait_Master_Sync(IRQ_TIMEOUT);
        debug("=============================接收主站周期数据==============================\n");
        rc = GM_Master_Receive(1, &PdoWarnInfo);
        if (PdoWarnInfo.PDOWarnFlag || PdoWarnInfo.DCWarnFlag){
            PdoWar_Info_Get(NULL, &PdoWarnInfo);
        }
        if (PdoWarnInfo.FlameTimeOutCount || PdoWarnInfo.CRCErrCount){
            // printf(" CRC错误次数:%d, 超时错误次数:%d\n", PdoWarnInfo.CRCErrCount, PdoWarnInfo.FlameTimeOutCount);
        }
        sem_post(&plc_sem);
        sem_post(&motion_sem);
        sem_post(&path_plan_sem);
    #endif
}

void Master_Run()
{
    while (!thread_exit_flag){
        // 判断硬件连线等错误
        Master_Hardware_Error(g_current_error_ctx);
        // 主站周期性中断信号
        Master_Wait_Sync();
        // 等待运控计算完成信号
        sem_wait(&moton_sem_exec_done);
        op_flag = 1;
        // 复位清除驱动器报警
        if(reset_sem)
        {
            for (int i = 0; i < 4; i++)
            {
                CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[i + 2].slave.statusWord, &status[i] , 2));// 获取状态字
                if((status[i] & 0x0f) == 0x08)
                {
                    // 清除错误
                    CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.Control_word, 0x80));
                }
                else if(dri_manager.drivers[i]->state == DRV_STATE_FAULT && (status[i] & 0x0f) != 0x08)
                {
                    dri_manager.drivers[i]->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;
                    dri_manager.drivers[i]->act_mode = DRV_MODE_NONE;
                    if(i = 0)
                    {
                        delAlarm(ALARM_AX, ALARM_ERROR, 33);
                    }
                    else
                    {
                        delAlarm(ALARM_AX, ALARM_ERROR, 29 + i);
                    }
                }
            }
            if (((status[0] & 0x0f) != 0x08 || rt_data->axis_data[3].status.is_disable)
                && ((status[1] & 0x0f) != 0x08 || rt_data->axis_data[0].status.is_disable)
                && ((status[2] & 0x0f) != 0x08 || rt_data->axis_data[1].status.is_disable)
                && ((status[3] & 0x0f) != 0x08 || rt_data->axis_data[2].status.is_disable))
            {
                reset_sem = false;
            }
        }

        // 进给轴驱动器设置成CSP模式 并上使能（理解：设置模式放在使能流程之前）
        // 驱动器对应控制字下发顺序：0x0006 >>> 0x0007 >>> 0x000f
        for (int i = 1; i < 4; i++)
        {
            // 每个周期都在读控制字
            CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[i + 2].slave.statusWord, &status[i] , 2));// 获取状态字信息
            CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[i + 2].slave.act_pos, &pos[i], 4));
            // 根据状态字设置控制字
            if((status[i] & 0x004f) == 0x0040) 
            {
                dri_manager.drivers[i]->state = DRV_STATE_SWITCHED_ON_DISABLE;
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.Modes_of_operation, 0x08)); // 设置操作模式, 08 设置位置模式 操作模式8位
                // 写控制字0x0006
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.Control_word, 0x0006));
            }
            else if((status[i] & 0x006f) == 0x0021)
            {
                // 写控制字0x0007
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.Control_word, 0x0007));
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.target_pos, pos[i]));
                pos_cmd[i] = pos[i];
                dri_manager.drivers[i]->state = DRV_STATE_READY_TO_SWITCHED_ON;
            }
            else if((status[i] & 0x006f) == 0x0023)
            {
                dri_manager.drivers[i]->state = DRV_STATE_SWITCHED_ON;
                // 写控制字0x000f
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 2].slave.Control_word, 0x000f));
            }
            else if((status[i] & 0x027f) == 0x0237)
            {
                dri_manager.drivers[i]->state = DRV_STATE_OPERATION_ENABLED;
            }
        }

        // 主轴驱动器设置成CSV模式 并上使能（理解：设置模式放在使能流程之前 即DRV_STATE_SWITCHED_ON_DISABLE后一个状态）
        // 驱动器状态变化：Ready to Switch On >>> Switched On >>> Operation Enabled
        // 驱动器对应控制字下发顺序：0x06 >>> 0x07 >>> 0x0f
        if(spindle.act_mode != DRV_MODE_NONE) // 实时启动时不会执行
        {
            // printf("************************************************************wcf\n");
            // 如果抽象主轴驱动器不是　DRV_MODE_DISABLED，就一直读主轴驱动器状态字．
            CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.statusWord, &status[0] , 2));// 获取主轴驱动器状态字
            //根据状态字设置控制字
            if(((status[0] & 0x004f) == 0x0040))
            {
                spindle.state = DRV_STATE_SWITCHED_ON_DISABLE;
                // 写控制字0x0006
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x0006)); // 主轴驱动器控制字写0x0006
                CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.act_pos, &pos[0], 4)); // 读主轴驱动器实际位置
            }
            else if((status[0] & 0x006f) == 0x0021)
            {
                spindle.state = DRV_STATE_READY_TO_SWITCHED_ON;
                if(spindle.act_mode == DRV_MODE_CSV)
                {
                    // 主轴驱动器模式控制6060写0x09，是将主轴驱动器设置成速度模式。 位置模式 模式控制6060要写0x08，速度模式 模式控制6060要写0x09
                    CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Modes_of_operation, 0x09));
                    // 写控制字0x0007
                    CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x0007));
                }
            }
            else if((status[0] & 0x006f) == 0x0023)
            {
                spindle.state = DRV_STATE_SWITCHED_ON;
                // 写控制字0x000f
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x000f));
            }
            else if((status[0] & 0x027f) == 0x0237)
            {
                spindle.state = DRV_STATE_OPERATION_ENABLED;
                // 到这里已经上完使能
            }
        }
        else
        {
            if(spindle.state != DRV_STATE_READY_TO_SWITCHED_ON)
            {
                spindle.state = DRV_STATE_NOT_READY_TO_SWITCH_ON;
                CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x00)); // 主轴驱动器控制字写0x00，主轴驱动器下使能
            }
        }

        // wcf test
        // 主轴驱动器设置成位置模式 并上使能
        // {
        //     CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.statusWord, &status[0], 2));// 获取状态字 每个周期都在读这个控制字
        //     CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.act_pos, &pos[0], 4));
        //     // 根据状态字设置控制字
        //     if((status[0] & 0x4f) == 0x40)
        //     {
        //         CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Modes_of_operation, 0x08)); // 设置操作模式, 08 设置位置模式
        //         CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x06));
        //         dri_manager.drivers[0]->state = DRV_STATE_SWITCHED_ON_DISABLE;
        //     }
        //     // 如果16位的状态字变成了下一阶段的结果，然后就发下一个控制字给驱动器，让驱动器的状态机继续变
        //     else if((status[0] & 0x6f) == 0x21) 
        //     {
        //         CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x07));
        //         CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.target_pos, pos[0]));
        //         pos_cmd[0] = pos[0];
        //         dri_manager.drivers[0]->state = DRV_STATE_READY_TO_SWITCHED_ON;
        //     }
        //     else if((status[0] & 0x6f) == 0x23)
        //     {
        //         dri_manager.drivers[0]->state = DRV_STATE_SWITCHED_ON;
        //         CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x0f));
        //     }
        //     else if((status[0] & 0x27f) == 0x237)
        //     {
        //         dri_manager.drivers[0]->state = DRV_STATE_OPERATION_ENABLED;
        //     }
        // }

        // wcf test 
        // uint32_t spindle_act_pos = 0;
        // int32_t spindle_act_vel = 0;
        // GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.act_pos, &spindle_act_pos, 4);
        // GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.act_speed, &spindle_act_vel, 4);
        // printf("spindle_pos%d spindle_act_vel%d \n", spindle_act_pos, spindle_act_vel);

        //读取驱动器警报
        for (int i = 0; i < 4; i++)
        {
            CHECK_FUNC(GM_TxPdoEntry_Read(g_device_data[i + 2].slave.error_code, &error_code[i], 2)); // 获取状态字
            if (i == 0)
            {
                if (error_code[0] != 0 && !rt_data->axis_data[3].status.is_disable)
                {
                    rt_data->axis_data[3].status.is_error = 1;
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 33);
                    if (res != -8)
                    LOG(LOG_LEVEL_ERR, "%s 轴驱动器报错", drv_names[i]);
                }
                else
                {
                    rt_data->axis_data[3].status.is_error = 0;
                }
            }
            else
            {
                if (error_code[i] != 0 && !rt_data->axis_data[i-1].status.is_disable)
                {
                    rt_data->axis_data[i-1].status.is_error = 1;
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 29 + i);
                    if (res != -8)
                    LOG(LOG_LEVEL_ERR, "%s 轴驱动器报错", drv_names[i]);
                }
                else
                {
                    rt_data->axis_data[i-1].status.is_error = 0;
                }
            }

            if (rt_data->axis_data[i].status.is_disable)
            {
                if(i == 3)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[2].slave.Control_word, 0x00));
                }
                else
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[i + 3].slave.Control_word, 0x00));
                }
            }
        }

        
        #ifndef DEBUG_MODE_ENABLED
        //检查驱动器是否存在报警
        Check_Driver_Error();
        #endif

        debug("=============================发送主站周期数据==============================\n");
        rc = GM_Master_Send(1);
    }
}

void Master_Reset()
{
    reset_sem = 1;
}

void Check_Driver_Error()
{
    if (!reset_sem)
    {
        for (int i = 0; i < 4; i++)
        {
            if (rt_data->axis_data[i].status.is_error)
            {
                if(i = 3)
                {
                    dri_manager.drivers[0]->state = DRV_STATE_FAULT;
                }
                else
                {
                    dri_manager.drivers[i-1]->state = DRV_STATE_FAULT;
                }
            }
        }
    }
}

// 主轴驱动器上使能
void spindle_driver_servo_on()
{
    uint16_t status_word;
    int ret;

    /*
     * CiA-402 的控制字必须按状态机逐步下发。一次调用只推进一个状态，
     * 下一步由后续周期读取到新的状态字后再执行。
     */
    status_word = 0;
    ret = GM_TxPdoEntry_Read(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.statusWord, &status_word, 2);
    if (ret < 0)
    {
        fprintf(stderr, "Error in function: GM_TxPdoEntry_Read(statusWord), rc = %d\n", ret);
        return;
    }

    if ((status_word & 0x4f) == 0x40)
    {
        CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x06));
        spindle.state = DRV_STATE_SWITCHED_ON_DISABLE;
    }
    else if ((status_word & 0x6f) == 0x21)
    {
        if (spindle.act_mode == DRV_MODE_CSV)
        {
            CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Modes_of_operation, 0x09));
        }
        CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x07));
        spindle.state = DRV_STATE_READY_TO_SWITCHED_ON;
    }
    else if ((status_word & 0x6f) == 0x23)
    {
        CHECK_FUNC(GM_RxPdoEntry_Write(g_device_data[DEV_ID_SPINDLE_DRIVER].slave.Control_word, 0x0f));
        spindle.state = DRV_STATE_SWITCHED_ON;
    }
    else if ((status_word & 0x27f) == 0x237 && spindle.state == DRV_STATE_SWITCHED_ON)
    {
        spindle.state = DRV_STATE_OPERATION_ENABLED;
    }
}

static void Master_Receive(){
    slavernum = Hal_Driver_Get_Count();
    status = (uint16_t*)malloc(slavernum * sizeof(uint16_t));
    pos = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    pos_cmd = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    error_code = (uint16_t*)malloc(slavernum * sizeof(uint16_t));
    
    // === 2.握手前读 ===
    debug("==============================握手前DMA读================================\n");
    rc = GM_Master_Receive(1, &PdoWarnInfo);
    if (PdoWarnInfo.PDOWarnFlag || PdoWarnInfo.DCWarnFlag ){
        PdoWar_Info_Get(NULL, &PdoWarnInfo);

    }
    if (PdoWarnInfo.FlameTimeOutCount || PdoWarnInfo.CRCErrCount){
        // udp通信错误，次数达到一定后会从警告升级为报警
        // printf(" CRC错误次数:%d, 超时错误次数:%d\n", PdoWarnInfo.CRCErrCount, PdoWarnInfo.FlameTimeOutCount);
    }

    if ( IO_Resource_Allocation_Scanf() != 0)
    {
        printf("驱动打开失败\n");
    }

    debug("==============================握手前DMA写================================\n");
    rc = GM_Master_Send(1);

    debug("====================与主站握手:GM_Master_OP_Valid()=======================\n");
    rc = GM_Master_OP_Valid();
    sem_post(&op_sem); // 通知其他线程主站已经进入op
}

static void Master_Hardware_Error(ErrorContext g_current_error_ctx){
    int res = 0;
    // printf("g_current_error_ctx.ErrorID = %x\n",g_current_error_ctx.ErrorID);
    // printf("g_current_error_ctx.Fms = %x\n",g_current_error_ctx.Fms);
    // printf("g_current_error_ctx.Model = %x\n",g_current_error_ctx.Model);
    // printf("g_current_error_ctx.Code = %x\n",g_current_error_ctx.Code);
    // printf("g_current_error_ctx.errslave = %x\n",g_current_error_ctx.errslave);
    switch (g_current_error_ctx.ErrorID)
    {
    case ECAT_INIT_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 100);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站初始化模块错误");
        break;
    case ECAT_CONTROL_PANEL_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 101);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "控制面板断连");
        break;
    case ECAT_IO_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 102);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "IO模块断连");
        break;
    case ECAT_SLAVE3_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 103);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "从站三断连");
        break;
    case ECAT_SLAVE4_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 104);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "从站四断连");
        break;
    case ECAT_SLAVE5_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 105);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "从站五断连");
        break;
    case ECAT_SLAVE6_CONNECT_LOST:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 106);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "从站六断连");
        break;
    case ECAT_TOPOLOGY_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 107);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站拓扑模块错误");
        break;
    case ECAT_ACTIVE_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 108);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站激活模块错误");
        break;
    case ECAT_DC_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 109);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站DC模块错误");
        break;
    case ECAT_PDO_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 110);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站PDO模块错误");
        break;
    case ECAT_RT_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 111);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站RT模块错误");
        break;
    case UNKOWN_ERROR:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 999);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "未知错误");
        break;
    default:
        break;
    }
}

static void Master_Error(int code)
{
    int res = 0;
    switch (code)
    {
    case -1000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 0);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "文件或设备打开失败");
        break;
    case -1001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 1);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "没有找到目标");
        break;
    case -1002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 2);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "空指针");
        break;
    case -1003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 3);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "存储空间不足");
        break;
    case -1004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 4);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "无效的参数");
        break;
    case -1005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 5);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "系统参数错误");
        break;
    case -1006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 6);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "驱动打开失败");
        break;
    case -1007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 7);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "内存分配失败");
        break;
    case -1008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 8);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "自旋锁分配失败");
        break;
    case -1009:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 9);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "传输驱动初始化失败");
        break;
    case -1010:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 10);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "信号量初始化失败");
        break;
    case -1011:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 11);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "超时");
        break;
    case -1012:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 12);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站无回复");
        break;
    case -1013:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 13);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "主站状态错误");
        break;
    case -2000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 20);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "无效的通信地Master_Run址");
        break;
    case -2001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 21);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "内存映射失败");
        break;
    case -2002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 22);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "内存回读失败");
        break;
    case -3000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 30);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "eni文件解析错误");
        break;
    case -3001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 31);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "传入的指针已有指向地址");
        break;
    case -3002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 32);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件中没有从站");
        break;
    case -3003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 33);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有厂商号");
        break;
    case -3004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 34);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有产品号");
        break;
    case -3005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 35);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有看门狗");
        break;
    case -3006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 36);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有syncyc");
        break;
    case -3007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 37);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站没有syncshift");
        break;
    case -3008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 38);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "ENI文件存在从站的某个sm没有方向说明（Tx or Rx）");
        break;
    case -4000:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 40);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "entry的长度无效");
        break;
    case -4001:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 41);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "文件或设备打开失败");
        break;
    case -4002:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 42);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "存在未完成的reg请求");
        break;
    case -4003:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 43);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "没有已发送的reg请求");
        break;
    case -4004:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 44);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "REG队列已满");
        break;
    case -4005:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 45);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "存在未完成的SDO请求");
        break;
    case -4006:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 46);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "没有已发送的reg请求");
        break;
    case -4007:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 47);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "SDObuffer的空间不足");
        break;
    case -4008:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 48);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "远程升级时计算次数与实际写入不符");
        break;
    case -9999:
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 999);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "未知错误");
        break;
    default:
        break;
    }
}

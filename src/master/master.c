#include "master.h"
#include "main_demo.h"
#include "rt_sem.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <stdio.h>
#include <unistd.h>
#include "log.h"

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
        goto err_close; \
} \
} while (0)

int master_error_code = 0;
bool reset_sem = 0;
static char* drv_names[] = {"X","Y","Z","主"};

static void Master_Receive();
static void Master_Error(int code);

int Master_Start() {
    char cwd[1024];
    char* eni_path = get_master_eni_path();
    master_error_code = ethercat_init(eni_path);
    if ( master_error_code < 0) {
        Master_Error(master_error_code);
        return -1;
    }
    Master_Receive();
    return 1;
}

static void Master_Receive(){
    slavernum = Hal_Driver_Get_Count();
    statis = (uint16_t*)malloc(slavernum * sizeof(uint16_t));
    pos = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    pos_cmd = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    error_code = (uint16_t*)malloc(slavernum * sizeof(uint16_t));
    
    // === 2.握手前读 ===
    debug("==============================握手前DMA读================================\n");
    rc = GM_Master_Receive(1, &PdoWarnInfo);
    CHECK_RC(rc, "握手前DMA读失败", err_close);
    if (PdoWarnInfo.PDOWarnFlag || PdoWarnInfo.DCWarnFlag ){
        PdoWar_Info_Get(NULL, &PdoWarnInfo);

    }
    if (PdoWarnInfo.FlameTimeOutCount || PdoWarnInfo.CRCErrCount){
        printf(" CRC错误次数:%d, 超时错误次数:%d\n", PdoWarnInfo.CRCErrCount, PdoWarnInfo.FlameTimeOutCount);
    }

    if ( IO_Resource_Allocation_Scanf() != 0)
    {
        printf("驱动打开失败\n");
    }
    // === 3.循环获取控制流程所需要的所有Entry ===
    //IO
    uint8_t mcp_slave_bias = 0;
    for(int i = 0; i < 11; ++i)
    {
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&MCPDI[i], slave_list, mcp_slave_bias, 0, i));
    }
    for(int i = 0; i < 7; ++i)
    {
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&MCPDO[i], slave_list, mcp_slave_bias, 0, i));
    }
    //MCP
    uint8_t io_slave_bias = 1;
    for (int i = 0; i < 1; i++)
    {
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&DI0[i], slave_list, i + io_slave_bias, 0, 0));
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&DI1[i], slave_list, i + io_slave_bias, 0, 1));

        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&DO0[i], slave_list, i + io_slave_bias, 0, 0));
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&DO1[i], slave_list, i + io_slave_bias, 0, 1));
    }
    //driver
    uint8_t drivers_slave_bias = 2;
#ifdef PLATFORM_1
    for (int i = 0; i < 3; i++) {
        GM_RxPdoEntry_Get_By_Pos(&ControlWord[i], slave_list , i + drivers_slave_bias, 0, 0);
        GM_RxPdoEntry_Get_By_Pos(&PurPos[i], slave_list, i + drivers_slave_bias, 0, 1);
        GM_RxPdoEntry_Get_By_Pos(&PurVel[i], slave_list, i + drivers_slave_bias, 0, 2);
        GM_RxPdoEntry_Get_By_Pos(&OperationMode[i], slave_list, i + drivers_slave_bias, 0, 4);

        GM_TxPdoEntry_Get_By_Pos(&StatusWord[i], slave_list, i + drivers_slave_bias, 0, 0);
        GM_TxPdoEntry_Get_By_Pos(&CurPos[i], slave_list, i + drivers_slave_bias, 0, 1);
        GM_TxPdoEntry_Get_By_Pos(&CurVel[i], slave_list, i + drivers_slave_bias, 0, 2);
        GM_TxPdoEntry_Get_By_Pos(&ErrorNumb[i], slave_list, i + drivers_slave_bias, 0, 3);
    }
    GM_RxPdoEntry_Get_By_Pos(&ControlWord[3], slave_list, 5, 0, 0);
    GM_RxPdoEntry_Get_By_Pos(&PurPos[3], slave_list,  5, 0, 2);
    GM_RxPdoEntry_Get_By_Pos(&OperationMode[3], slave_list, 5, 0, 1);

    GM_TxPdoEntry_Get_By_Pos(&ErrorNumb[3], slave_list, 5, 0, 0);
    GM_TxPdoEntry_Get_By_Pos(&StatusWord[3], slave_list, 5, 0, 1);
    GM_TxPdoEntry_Get_By_Pos(&CurPos[3], slave_list, 5, 0, 3);

#else
    for (int i = 0; i < 4; i++) {
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurVel[i], slave_list, i + drivers_slave_bias, 1, 0));
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&ControlWord[i], slave_list, i + drivers_slave_bias, 0, 0));     // 获取控制字entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&OperationMode[i], slave_list, i + drivers_slave_bias, 1, 1));   // 获取操作模式entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurPos[i], slave_list, i + drivers_slave_bias, 0, 1));

        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&StatusWord[i], slave_list, i + drivers_slave_bias, 0, 0));      // 获取状态字entry信息
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurPos[i], slave_list, i + drivers_slave_bias, 0, 1));          // 获取当前位置entry信息
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurVel[i], slave_list, i + drivers_slave_bias, 1, 0));
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&ErrorNumb[i], slave_list, i + drivers_slave_bias, 2, 0));
    }
#endif

    debug("==============================握手前DMA写================================\n");
    rc = GM_Master_Send(1);
    CHECK_RC(rc, "握手前DMA写失败", err_close);

    debug("====================与主站握手:GM_Master_OP_Valid()=======================\n");
    rc = GM_Master_OP_Valid();
    CHECK_RC(rc, "与主站握手失败", err_close);

    sem_post(&op_sem); // 通知其他线程主站已经进入op

err_close:
    Master_Error(rc);
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
        CHECK_RC(rc, "等待周期中断失败", err_close);
        debug("=============================接收主站周期数据==============================\n");
        rc = GM_Master_Receive(1, &PdoWarnInfo);
        CHECK_RC(rc, "握手前DMA读失败", err_close);
        if (PdoWarnInfo.PDOWarnFlag || PdoWarnInfo.DCWarnFlag){
            PdoWar_Info_Get(NULL, &PdoWarnInfo);
        }
        if (PdoWarnInfo.FlameTimeOutCount || PdoWarnInfo.CRCErrCount){
            printf(" CRC错误次数:%d, 超时错误次数:%d\n", PdoWarnInfo.CRCErrCount, PdoWarnInfo.FlameTimeOutCount);
        }
        sem_post(&plc_sem);
        sem_post(&motion_sem);
        sem_post(&path_plan_sem);
    #endif
err_close:
    Master_Error(rc);
}

void Master_Run()
{
    while (!thread_exit_flag){
        // 主站周期性中断信号
        Master_Wait_Sync();
        sem_wait(&moton_sem_exec_done);
        op_flag = 1;
        //复位清除驱动器报警
        if(reset_sem)
        {
            for (int i = 0; i < 4; i++)
            {
                CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[i], &statis[i] , 2));// 获取状态字
                if((statis[i] & 0x0f) == 0x08)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x80));
                    dri_manager.drivers[i]->state = DRV_STATE_NOT_READY;
                    dri_manager.drivers[i]->mode = DRV_MODE_DISABLED;
                }
            }
            if (((statis[0] & 0x0f) != 0x08 || rt_data->axis_data[3].axis_status.is_disable)
                && ((statis[1] & 0x0f) != 0x08 || rt_data->axis_data[0].axis_status.is_disable)
                && ((statis[2] & 0x0f) != 0x08 || rt_data->axis_data[1].axis_status.is_disable)
                && ((statis[3] & 0x0f) != 0x08 || rt_data->axis_data[2].axis_status.is_disable))
            {
                reset_sem = false;
            }
        }
        //进给轴上电
        for (int i = 1; i < 4; i++)
        {
            CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[i], &statis[i] , 2));// 获取状态字
            CHECK_FUNC(GM_TxPdoEntry_Read(CurPos[i], &pos[i], 4));
            // 根据状态字设置控制字
            if((statis[i] & 0x4f) == 0x40)
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(OperationMode[i], 0x08)); // 设置操作模式
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x06));
                dri_manager.drivers[i]->state = DRV_STATE_SWITCHED_ON_DISABLE;
            }
            else if((statis[i] & 0x6f) == 0x21)
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x07));
                CHECK_FUNC(GM_RxPdoEntry_Write(PurPos[i], pos[i]));
                pos_cmd[i] = pos[i];
                dri_manager.drivers[i]->state = DRV_STATE_READY_TO_SWITCHED_ON;
            }
            else if((statis[i] & 0x6f) == 0x23)
            {
                dri_manager.drivers[i]->state = DRV_STATE_SWITCHED_ON;
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x0f));
            }
            else if((statis[i] & 0x27f) == 0x237)
            {
                dri_manager.drivers[i]->state = DRV_STATE_OPERATION_ENABLED;
            }
        }
        //主轴上电
        if(spindle.mode != DRV_MODE_DISABLED)
        {
            CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[0], &statis[0] , 2));// 获取状态字
            //根据状态字设置控制字
            if(((statis[0] & 0x4f) == 0x40))
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x06));
                CHECK_FUNC(GM_TxPdoEntry_Read(CurPos[0], &pos[0], 4));
                spindle.state = DRV_STATE_SWITCHED_ON_DISABLE;
            }
            else if((statis[0] & 0x6f) == 0x21)
            {
                if(spindle.mode == DRV_MODE_CSV)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(OperationMode[0], 0x09));// 设置操作模式
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x07));
                }
                spindle.state = DRV_STATE_READY_TO_SWITCHED_ON;
            }
            else if((statis[0] & 0x6f) == 0x23)
            {
                spindle.state = DRV_STATE_SWITCHED_ON;
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x0f));
            }
            else if((statis[0] & 0x27f) == 0x237 && (spindle.state == DRV_STATE_SWITCHED_ON))
            {
                spindle.state = DRV_STATE_OPERATION_ENABLED;
            }
        }
        else
        {
            if(spindle.state != DRV_STATE_READY_TO_SWITCHED_ON)
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x00));
            }
        }

        //读取驱动器警报
        for (int i = 0; i < 4; i++)
        {
            CHECK_FUNC(GM_TxPdoEntry_Read(ErrorNumb[i], &error_code[i], 2)); // 获取状态字
            dri_manager.drivers[i]->state = DRV_STATE_FAULT;
            if (i == 0)
            {
                if (error_code[0] != 0 && !rt_data->axis_data[3].axis_status.is_disable)
                {
                    rt_data->axis_data[3].axis_status.is_error = 1;
                }
                else
                {
                    rt_data->axis_data[3].axis_status.is_error = 0;
                }
            }
            else
            {
                // printf("i:%d %d \n ",i,rt_data->axis_data[i-1].axis_status.is_disable);
                if (error_code[i] != 0 && !rt_data->axis_data[i-1].axis_status.is_disable)
                {
                    rt_data->axis_data[i-1].axis_status.is_error = 1;
                }
                else
                {
                    rt_data->axis_data[i-1].axis_status.is_error = 0;
                }
                // printf("rt_data->axis_data[i-1].axis_status.is_error:%d\n",rt_data->axis_data[i-1].axis_status.is_error);
            }

            if (rt_data->axis_data[i].axis_status.is_disable)
            {
                if(i == 3)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x00));
                }
                else
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i + 1], 0x00));
                }
            }
        }

        debug("=============================发送主站周期数据==============================\n");
        rc = GM_Master_Send(1);
        CHECK_RC(rc, "发送主站周期数据失败", err_close);
    }
err_close:
    Master_Error(rc);
}

void Master_Reset()
{
    reset_sem = 1;
}

void Check_Driver_Error()
{
    for (int i = 0; i < 4; i++)
    {
        if(rt_data->axis_data[i].axis_status.is_error)
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
            if(res != -8) 
            LOG(LOG_LEVEL_ERR, "%s 轴驱动器报错",drv_names[i]);
        }   
        if(!rt_data->axis_data[i].axis_status.is_error)
        {
            int res = delAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
            if(res == 0)
            LOG(LOG_LEVEL_INFO, "%s 轴驱动器报错已清除",drv_names[i]);
        }  
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
        LOG(LOG_LEVEL_ERR, "无效的通信地址");
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
        res = setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 99);
        if(res == -8) return;
        LOG(LOG_LEVEL_ERR, "未知错误");
        break;
    default:
        break;
    }
}

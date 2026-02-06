#include "master.h"
#include "main_demo.h"
#include "rt_sem.h"
#include "global.h"

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

bool sp_start_sem = false;
bool feed_start_sem = false;

void Master_Run()
{
#ifdef MASTER_RUN_ON
    int slavernum = Hal_Driver_Get_Count();
    uint32_t* statis = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    uint32_t* pos = (uint32_t*)malloc(slavernum * sizeof(uint32_t));
    uint32_t* pos_cmd = (uint32_t*)malloc(slavernum * sizeof(uint32_t));

    // === 1.定义变量 ===
    int rc = 0;
    PdoWarn PdoWarnInfo; //pdo警告信息结构体
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
    uint8_t mcp_slave_bias = 0;
    for(int i = 0; i < 11; ++i)
    {
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&MCPDI[i], slave_list, mcp_slave_bias, 0, i));
    }
    for(int i = 0; i < 7; ++i)
    {
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&MCPDO[i], slave_list, mcp_slave_bias, 0, i));
    }
    uint8_t io_slave_bias = 1;
    for (int i = 0; i < 1; i++)
    {
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&DI0[i], slave_list, i + io_slave_bias, 0, 0));
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&DI1[i], slave_list, i + io_slave_bias, 0, 1));

        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&DO0[i], slave_list, i + io_slave_bias, 0, 0));
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&DO1[i], slave_list, i + io_slave_bias, 0, 1));
    }
    uint8_t drivers_slave_bias = 2;
    /***************************根据位置索引Entry**************************/
    for (int i = 0; i < 4; i++) {
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&StatusWord[i], slave_list, i + drivers_slave_bias, 0, 0));      // 获取状态字entry信息
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurPos[i], slave_list, i + drivers_slave_bias, 0, 1));          // 获取当前位置entry信息
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurVel[i], slave_list, i + drivers_slave_bias, 1, 0));

        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurVel[i], slave_list, i + drivers_slave_bias, 1, 0));
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&ControlWord[i], slave_list, i + drivers_slave_bias, 0, 0));     // 获取控制字entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&OperationMode[i], slave_list, i + drivers_slave_bias, 1, 1));   // 获取操作模式entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurPos[i], slave_list, i + drivers_slave_bias, 0, 1));
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&ErrorNumb[i], slave_list, i + drivers_slave_bias, 2, 0));

    }
    debug("==============================握手前DMA写================================\n");
    rc = GM_Master_Send(1);
    CHECK_RC(rc, "握手前DMA写失败", err_close);

    debug("====================与主站握手:GM_Master_OP_Valid()=======================\n");
    rc = GM_Master_OP_Valid();
    CHECK_RC(rc, "与主站握手失败", err_close);

    sem_post(&op_sem); // 通知其他线程主站已经进入op
#endif

    while (!exit_flag){ // 进入控制逻辑循环
#ifdef MASTER_RUN_ON
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
#endif
#ifndef MASTER_RUN_ON

        post_rt_sem_all();
op_flag = 1;
        usleep(1000);
#else
        sem_post(&plc_sem);
        if(!feed_start_sem)
        {
            for (int i = 1; i < 4; i++)
            {
                CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[i], &statis[i] , 4));// 获取状态字
                CHECK_FUNC(GM_TxPdoEntry_Read(CurPos[i], &pos[i], 4));
                // 根据状态字设置控制字
                if(((statis[i] & 0x4f) == 0x40))
                {
                    printf(" CHECK_FUNC(GM_RxPdoEntry_Write%d\n",i);
                    CHECK_FUNC(GM_RxPdoEntry_Write(OperationMode[i], 0x08)); // 设置操作模式
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x06));
                }
                else if((statis[i] & 0x6f) == 0x21)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x07));
                    CHECK_FUNC(GM_RxPdoEntry_Write(PurPos[i], pos[i]));
                    pos_cmd[i] = pos[i];
                }
                else if((statis[i] & 0x6f) == 0x23)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x0f));
                }
                else if((statis[i] & 0x27f) == 0x237)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x1f));
                    feed_start_sem = 1;
                    op_flag =1;
                    printf("op_flag =1;\n");

                }
            }
        }
        if(op_flag)
        {
            sem_post(&motion_sem);
            sem_post(&path_plan_sem);
            sem_wait(&moton_sem_exec_done);
        }

        if(!sp_start_sem)
        {
            CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[0], &statis[0] , 4));// 获取状态字
            //根据状态字设置控制字
            if(((statis[0] & 0x4f) == 0x40))
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x06));
                CHECK_FUNC(GM_TxPdoEntry_Read(CurPos[0], &pos[0], 4));
            }
            else if((statis[0] & 0x6f) == 0x21)
            {
                if(spindle.mode == DRV_MODE_CSV)
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(OperationMode[0], 0x09));// 设置操作模式
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x07));
                }
                spindle.state = DRV_STATE_NOT_READY;
            }
            else if((statis[0] & 0x6f) == 0x23)
            {
                spindle.state = DRV_STATE_SWITCHED_ON;
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x0f));
            }
            else if((statis[0] & 0x27f) == 0x237 && (spindle.state = DRV_STATE_SWITCHED_ON))
            {
                spindle.state = DRV_STATE_OPERATION_ENABLED;
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x1f));
                sp_start_sem = 1;
            }
            if(spindle.state != DRV_STATE_SWITCHED_ON)
            {
                sp_start_sem = 0;
            }
        }
        if(spindle.mode == DRV_MODE_DISABLED)
        {
            if(spindle.state != DRV_STATE_NOT_READY)
            {
                CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[0], 0x00));
            }
        }
        debug("=============================发送主站周期数据==============================\n");
        rc = GM_Master_Send(1);
        CHECK_RC(rc, "发送主站周期数据失败", err_close);
#endif

    }

    return 1;
err_close:
    printf("pdo线程不正常退出");
    exit_flag = 1;
    return -1;
}

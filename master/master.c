#include "master.h"
#include "rt_sem.h"

#ifdef __DeveloperMode__
#define debug printf
#else
#define debug(...)
#endif
// ===============宏定义=================
#define WAIT_TIME 120
#define IRQ_TIMEOUT 5000
#define PARAM_CHANG 0
#define MAN_FMMU 0
#define OP_PRE_REG_EN 0
#define RT_POLICY SCHED_FIFO
#define MAIN_PRIORITY 40
#define REG_PRIORITY 50
#define SDO_PRIORITY 50
#define TEST_MODE_OP_INTER 0
// ===============错误检查宏=================
#define CHECK_RC(rc, info, label) \
do { \
        if ((rc) < 0) { \
            printf("%s, rc = %d\n", info, rc); \
            goto label; \
    } \
} while (0)

#define CHECK_FUNC(func) do { \
    int ret = (func); \
    if (ret < 0) { \
        fprintf(stderr, "Error in function: %s, rc = %d\n", #func, ret); \
        goto err_close; \
} \
} while (0)

    // ==============全局变量定义===============
    sem_t exit_sem;
    sem_t op_sem;
//sem_t master_sem;
int InterruptFlag = 0;
Slave_info* slave_list = NULL;

int slave_num = 0;

simple_tp_t simple_tp[5];

// ================错误处理函数=================
void err_call_back() {
    Err_info errinfo;
    Err_Code_Get(&errinfo);

    Err_Info_Get(errinfo, NULL);
    exit_flag = 1;
    InterruptFlag = 1;
    sem_post(&exit_sem);
}

// 提前声明线程和函数

void* reg_thread_func();
void* sdo_thread_func();
Fmmu_Manul* FMMU_Manul_Set_Func();

// ================ 新增：初始化函数（供外部调用）=================
int ethercat_init() {
    int rc = 0;
    uint32_t CRCCount = 0;
    uint32_t TimeOutCount = 0;
    // 1. 初始化信号量（初始值=0，表示不可用）
    //sem_init(&master_sem, 0, 0);

    rc = sem_init(&exit_sem, 0, 0);
    CHECK_RC(rc, "初始化信号量失败", err_close);
    rc = sem_init(&op_sem, 0, 0);
    CHECK_RC(rc, "初始化信号量失败", err_close);

    printf("================ 申请主站资源:GM_Resource_Allocation ==================\n");
    rc = GM_Resource_Allocation();
    CHECK_RC(rc, "申请主站资源失败", err_close);

    printf("================ 获取主站版本号:GM_Get_Version ==================\n");
    _Version_ MasterVersionCode = GM_Get_Version();
    printf("主站版本号：%d.%d.%\n", MasterVersionCode.FPGA_Version_Main, MasterVersionCode.FPGA_Version_Sec, MasterVersionCode.API_Version);

    printf("================ 申请报错回调函数:Err_Fun_Register ==================\n");
    rc = Err_Fun_Register(err_call_back);
    CHECK_RC(rc, "申请报错回调函数失败", err_close);

    printf("==================== 主站初始化:GM_Master_Init ========================\n");
    rc = GM_Master_Init();
    CHECK_RC(rc, "主站初始化失败", err_close);

    if (PARAM_CHANG) {
        _MASTER_PARAM_SRTUCT_.P_AL_STATE_CHANGE_TIMEOUT = 10000;
        printf("==================== 主站参数修改:GM_Master_Param_Change ========================\n");
        rc = GM_Master_Param_Change();
        CHECK_RC(rc, "主站参数修改", err_close);
    }

    printf("===================== 主站开始:GM_Master_Start ========================\n");
    rc = GM_Master_Start(10, &InterruptFlag);
    CHECK_RC(rc, "主站开始失败", err_close);

    slave_num = GM_Slave_Num_Get();
    if (slave_num < 0) {
        printf("slave num get error\n");
        goto err_close;
    } else {
        printf("slave num is %d\n", slave_num);
    }

    int interrupt = 0;
    const size_t buffer_size = 4 * 1024 * 1024;
    char *eeprom_memory = (char *)malloc(buffer_size);
    if (!eeprom_memory) {
        rc = -1;
        goto err_close;
    }

    printf("=================获取EEPROM:GM_EEPROM_Get=================\n");
    for (int i = 0; i < slave_num; i++) {
        memset(eeprom_memory, 0, buffer_size);
        rc = GM_EEPROM_Get(i, eeprom_memory, 10, &interrupt);
        CHECK_RC(rc, "获取EEPROM失败", err_close);
        int ID = eeprom_memory[16] + (eeprom_memory[17] << 8) + (eeprom_memory[18] << 16) + (eeprom_memory[19] << 24);
        int CODE = eeprom_memory[20] + (eeprom_memory[21] << 8) + (eeprom_memory[22] << 16) + (eeprom_memory[23] << 24);
        printf("slave %d : Vender ID is 0x%x, ProductCode is 0x%x\n", i + 1, ID, CODE);
    }
    free(eeprom_memory);

    printf("==================== 解析ENI文件:GM_ENI_Parse =========================\n");
    rc = GM_ENI_Parse(XMLPATH, &slave_list);
    CHECK_RC(rc, "ENI文件解析失败", err_close);

    printf("=============== 计算PDO映射:GM_Calculate_Config_Info ==================\n");
    rc = GM_Calculate_Config_Info(&slave_list);
    CHECK_RC(rc, "计算PDO映射失败", err_close);

    printf("=============== 设置主站通讯周期:GM_Master_Set_Cycle ==================\n");
    rc = GM_Master_Set_Cycle(CYCLE_TIME);
    CHECK_RC(rc, "设置主站通讯周期失败", err_close);

    if (DC_EN) {
        printf("=============== 使能DC信号:GM_DC_Enable ==================\n");
        rc = GM_DC_Enable();
        CHECK_RC(rc, "使能DC信号失败", err_close);
    }

    if (REG_PTH_EN) {
        printf("=============== 正在申请寄存器资源 ==================\n");
        int reg_num = 2;
        uint32_t mem_size[2] = {6, 6};
        rc = GM_Reg_Resource_Init(reg_num, mem_size);
        CHECK_RC(rc, "申请寄存器资源失败", err_close);
    }

    if (SDO_PTH_EN) {
        printf("=============== 正在申请SDO资源 ==================\n");
        rc = GM_Sdo_Datagram_Enable();
        CHECK_RC(rc, "SDO功能使能失败", err_close);
    }

    Fmmu_Manul* fmmu_manul_list = NULL;
    if (MAN_FMMU)
        fmmu_manul_list = FMMU_Manul_Set_Func();

    printf("========== 配置信息下发并激活:GM_Config_Download_And_Active ============\n");
    rc = GM_Config_Download_And_Active(slave_list, &CRCCount, &TimeOutCount, WAIT_TIME, &InterruptFlag, fmmu_manul_list);
    CHECK_RC(rc, "配置信息下发失败", err_close);

    if (CRCCount || TimeOutCount) {
        printf("op前数据帧出现错误: CRC错误次数:%d, 超时错误次数:%d\n", CRCCount, TimeOutCount);
    }

    if (MAN_FMMU)
        Fmmu_Manul_Free(fmmu_manul_list);

    printf("========== 等待主站进入OP:GM_Master_Wait_OP ============\n");
    rc = GM_Master_Wait_OP(WAIT_TIME, &InterruptFlag);
    CHECK_RC(rc, "等待主站进入OP", err_close);



    if (REG_PTH_EN) {
        pthread_t reg_thread;
        rc = pthread_create(&reg_thread, NULL, reg_thread_func, NULL);
        CHECK_RC(rc, "reg线程创建失败", err_close);
        pthread_detach(reg_thread);
    }

    if (SDO_PTH_EN) {
        pthread_t sdo_thread;
        rc = pthread_create(&sdo_thread, NULL, sdo_thread_func, NULL);
        CHECK_RC(rc, "sdo线程创建失败", err_close);
        pthread_detach(sdo_thread);
    }

    return 0;

err_close:
    ethercat_close(); // 清理资源
    return rc;
}

int ethercat_close() {
    int rc = 0;
    exit_flag = 1;
    InterruptFlag = 1;
    sem_post(&exit_sem);

    printf("=================== 释放从站信息链表:GM_Free_PDO_Map ==================\n");
    if (slave_list) {
        rc = GM_Free_PDO_Map(slave_list);
        if (rc < 0){
            printf("%s,rc = %d\n", "释放从站信息链表失败", rc);
        }
        else
        {
            printf("=================== 已释放从站信息链表:GM_Free_PDO_Map ==================\n");

        }
    }
    printf("===================== 主站关闭:GM_Master_Close =======================\n");
    rc = GM_Master_Close();
    if (rc < 0){
        printf("%s,rc = %d\n", "主站关闭失败", rc);
    }
    printf("================== 主站资源释放:GM_Resource_Release ===================\n");
    rc = GM_Resource_Release();
    //rc = IO_Resource_Release();
    if (rc < 0){
        printf("%s,rc = %d\n", "资源释放失败", rc);
    }
    sem_destroy(&exit_sem);
    sem_destroy(&op_sem);
    //sem_destroy(&master_sem);
    return rc;
}

// ====================================================PDO线程（高优先级）====================================================
// ====================================================REG线程====================================================
void* reg_thread_func(void *){
    int rc = 0;
    uint32_t WKC[2];    // 用来存储wkc
    uint32_t buffer[2]; // 用来传递读取值或写入值
    uint8_t warn[2];   // 用来存储警告
    unsigned short int AL[2],error_code[2];
    RegRequests reg_requests_list[2];
    reg_requests_list[0] = (RegRequests){0, 0, 0x130, 6, &WKC[0], &buffer[0], &warn[0]};
    reg_requests_list[1] = (RegRequests){0, 1, 0x130, 6, &WKC[1], &buffer[1], &warn[1]};
    while (!exit_flag){
        rc = GM_Reg_Request_Send(reg_requests_list, 2);
        CHECK_RC(rc, "寄存器数据发送失败", err_close);
        rc = GM_Reg_Request_Receive(5);
        CHECK_RC(rc, "寄存器数据接收失败", err_close);
        for (size_t i = 0; i < 2 ; i++)
        {
            AL[i] = reg_requests_list[i].data[0] + (reg_requests_list[i].data[1] << 8);
            error_code[i] = reg_requests_list[i].data[4] + (reg_requests_list[i].data[5] << 8);
            //printf("slave %d: al state is 0x%x\n", i + 1, AL[i]);
            if(AL[i] != 0x08){
                printf("warning : op down\n");
                printf("slave %d: al error code is %x\n", i + 1, error_code[i]);
            }
        }
        sleep(1);
    }
err_close:
    return NULL;
}
// ====================================================SDO线程====================================================
int write_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry) {
    int rc = 0;
    int cnt = 0;

    // 严格按结构体定义初始化
    SdoRequests sdo_request = {
        .WR = 1,                  // 0 = 写操作
        .SlavePos = slave_pos,            // 从站位置（0表示当前从站）
        .Index = index,           // 索引号
        .SubIndex = subindex,     // 子索引号
        .Size = (uint16_t)data_size, // 传输大小（转换为uint16_t）
        .data = (uint8_t*)data,   // 数据指针（转换为uint8_t*）
        .status = 0,              // 初始状态
        .Warn = 0,
        .WarnPra1 = 0,
        .WarnPra2 = 0
    };

    // 发送初始请求
    rc = GM_Sdo_Request_Send(sdo_request);
    //rc = GM_Sdo_Receive(&sdo_request, 5);
    while(sdo_request.status != P_SDO_SUCCESS){
        rc = GM_Sdo_Receive(&sdo_request, 5);
        if(sdo_request.status == P_SDO_ERROR){
            printf("sdo send error %d\n",sdo_request.status);
            rc = GM_Sdo_Request_Send(sdo_request);
        }
        printf("sdo send status is %d\n",sdo_request.status);
    }
    return 0;
}
// 通用SDO读取函数（严格匹配结构体定义）
int read_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry) {
    int rc = 0;
    int cnt = 0;

    // 严格按结构体定义初始化
    SdoRequests sdo_request = {
        .WR = 0,                  // 0 = 读操作
        .SlavePos = slave_pos,            // 从站位置（0表示当前从站）
        .Index = index,           // 索引号
        .SubIndex = subindex,     // 子索引号
        .Size = (uint16_t)data_size, // 传输大小（转换为uint16_t）
        .data = (uint8_t*)data,   // 数据指针（转换为uint8_t*）
        .status = 0,              // 初始状态
        .Warn = 0,
        .WarnPra1 = 0,
        .WarnPra2 = 0
    };

    // 发送初始请求
    // 发送初始请求
    rc = GM_Sdo_Request_Send(sdo_request);
    //rc = GM_Sdo_Receive(&sdo_request, 5);
    while(sdo_request.status != P_SDO_SUCCESS){
        rc = GM_Sdo_Receive(&sdo_request, 5);
        if(sdo_request.status == P_SDO_ERROR){
            printf("sdo send error %d\n",sdo_request.status);
            rc = GM_Sdo_Request_Send(sdo_request);
        }
        printf("sdo send status is %d\n",sdo_request.status);
    }
    //printf("SDO warn is%d, %d, %d\n", sdo_request.Warn, sdo_request.WarnPra1, sdo_request.WarnPra2);
    //SdoWar_Info_Get(NULL, &sdo_request);        // 检测是否存在警告信息

    return 0;
}

// 线程函数（使用封装函数）
void *sdo_thread_func(void *) {
    printf("sdo thread create\n");
    char software_version[100] = {0};
    for(int i = 0; i < 2; i++){
        memset(software_version, 0, sizeof(software_version));
        if(read_sdo_index(i, 0x100a, 0x00, software_version, 100, 10) == 0){
            printf("slave %d :software_version is %s\n", i + 1, software_version);
        }
    }
    char hardware_version[100] = {0};
    for(int i = 0; i < 2; i++){
        memset(hardware_version, 0, sizeof(hardware_version));
        if(read_sdo_index(i, 0x1009, 0x00, hardware_version, 100, 10) == 0){
            printf("slave %d :hardware_version is %s\n", i + 1, hardware_version);
        }
    }
    return NULL;
}

// ====================================================手动设置FMMU函数============================================
Fmmu_Manul* FMMU_Manul_Set_Func(){
    // 示例：初始化2个从设备，每个从设备的FMMU数量分别为3和2
    uint32_t num_slaves = 7;                    // 从站数量
    uint32_t every_slave_fmmu_num[] = {2,2,2,2,2,2,2};   // 每个从站所需要的fmmu

    // 调用初始化函数
    Fmmu_Manul* fmmu_manul = Fmmu_Manul_Init(num_slaves, every_slave_fmmu_num);
    if (!fmmu_manul) {
        printf("初始化失败！\n");
        return NULL;
    }
    // 示例：访问并打印FMMU数据
    printf("总从设备数量: %d\n", fmmu_manul->total_slave_number);
    fmmu_manul->total_Rxpdo_addr = 0;       // 收fmmu起始地址
    fmmu_manul->total_Rxpdo_size = 20;       // 收fmmu总长度
    fmmu_manul->total_Txpdo_addr = 20;       // 发fmmu起始地址
    fmmu_manul->total_Txpdo_size = 20;       // 发fmmu总长度
    uint32_t fmmu_log_start_addr[7][2] = {{0x0000, 0x0014},
        {0x0000, 0x0014},
        {0x0010, 0x0014},
        {0x0010, 0x0024},
        {0x0010, 0x0024},
        {0x0010, 0x0026},
        {0x0012, 0x0026},};
    uint32_t fmmu_data_size[7][2]      = {{0x0000, 0x0000},
        {0x0010, 0x0000},
        {0x0000, 0x0010},
        {0x0000, 0x0000},
        {0x0000, 0x0002},
        {0x0002, 0x0000},
        {0x0002, 0x0002},};
    uint32_t fmmu_log_start_bit[7][2]  = {{0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},};
    uint32_t fmmu_log_end_bit[7][2]    = {{0x0007, 0x0007},
        {0x0007, 0x0007},
        {0x0007, 0x0007},
        {0x0007, 0x0007},
        {0x0007, 0x0007},
        {0x0007, 0x0007},
        {0x0007, 0x0007},};
    uint32_t fmmu_phy_start_addr[7][2] = {{0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x1700, 0x1C00},
        {0x1100, 0x1400},
        {0x1100, 0x1400},
        {0x1100, 0x1400},};
    uint32_t fmmu_start_bit[7][2]      = {{0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},
        {0x0000, 0x0000},};
    uint32_t fmmu_dir[7][2]            = {{0x0002, 0x0001},
        {0x0002, 0x0001},
        {0x0002, 0x0001},
        {0x0002, 0x0001},
        {0x0002, 0x0001},
        {0x0002, 0x0001},
        {0x0002, 0x0001},};
    uint32_t fmmu_enable[7][2]         = {{0x0000, 0x0000},
        {0x0001, 0x0000},
        {0x0000, 0x0001},
        {0x0000, 0x0000},
        {0x0000, 0x0001},
        {0x0001, 0x0000},
        {0x0001, 0x0001},};
    for (size_t i = 0; i < 7; i++)
    {
        for (size_t j = 0; j < 2; j++)
        {
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_num = j;
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_start_addr = fmmu_log_start_addr[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_data_size = fmmu_data_size[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_start_bit = fmmu_log_start_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_end_bit = fmmu_log_end_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_phy_start_addr = fmmu_phy_start_addr[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_start_bit = fmmu_start_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_dir = fmmu_dir[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_enable = fmmu_enable[i][j];
        }

    }
    // 遍历每个从设备
    for (uint32_t i = 0; i < num_slaves; i++) {
        Fmmu_Manul_Slave* slave = &fmmu_manul->Fmmu_manul_slave_list[i];
        printf("从设备 %d:\n", i);
        printf("  FMMU总数量: %d\n", slave->fmmu_total_quantity);

        // 遍历该从设备的每个FMMU单元
        for (uint32_t j = 0; j < slave->fmmu_total_quantity; j++) {
            Fmmu_Manul_Unit* unit = &slave->Fmmu_manul_unit_list[j];
            printf("    FMMU单元 %d:\n", j);
            printf("      编号: %d\n", unit->fmmu_num);
            printf("      逻辑起始地址: 0x%x\n", unit->fmmu_log_start_addr);
            printf("      数据长度: %d\n", unit->fmmu_data_size);
            printf("      逻辑起始位: %d\n", unit->fmmu_log_start_bit);
            printf("      逻辑结束位: %d\n", unit->fmmu_log_end_bit);
            printf("      物理起始地址: 0x%x\n", unit->fmmu_phy_start_addr);
            printf("      起始位: %d\n", unit->fmmu_start_bit);
            printf("      类型: %d\n", unit->fmmu_dir);
            printf("      使能: %d\n", unit->fmmu_enable);
            printf("      保留字段: %d, %d, %d\n", unit->fmmu_reserved1, unit->fmmu_reserved2, unit->fmmu_reserved3);
        }
    }
    return fmmu_manul;
}

// ====================================================PDO线程（高优先级）====================================================
void master_run(){
    // === 1.定义变量 ===
    int rc = 0;
    PdoWarn PdoWarnInfo; //pdo警告信息结构体

    uint32_t statis[SLAVENUM];
    uint32_t pos[SLAVENUM];
    int32_t pos_act[SLAVENUM] = {0};
    uint16_t  start_statis = 0;
    bool start_sem =false;

    uint32_t axisandrate = 0;
    uint32_t current_count = 0; // 手轮计数器
    int rate = 0;     // 轴选和倍率
    int axis = 0;     // 轴选
    int last_axis = 0;     // 上一个周期轴选
    int last_rate = 0;     // 上一个周期轴选
    bool mpg_statis = true; //true为复位状态

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
        goto close;
    }

    // === 3.循环获取控制流程所需要的所有Entry ===
    for (int i = 0; i < SLAVENUM; i++){
        /***************************根据位置索引Entry**************************/
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&StatusWord[i], slave_list, i, 0, 0));      // 获取状态字entry信息
        CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurPos[i], slave_list, i, 0, 1));          // 获取当前位置entry信息
        //CHECK_FUNC(GM_TxPdoEntry_Get_By_Pos(&CurVec[i], slave_list, i, 1, 0));          // 获取当前速度entry信息

        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&ControlWord[i], slave_list, i, 0, 0));     // 获取控制字entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&OperationMode[i], slave_list, i, 1, 1));   // 获取操作模式entry信息
        CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurPos[i], slave_list, i, 0, 1));          // 获取目标位置entry信息
        //CHECK_FUNC(GM_RxPdoEntry_Get_By_Pos(&PurVec[i], slave_list, i, 1, 0));          // 获取目标速度entry信息
        /***************************根据索引号索引Entry**************************/
        /*待测*/
        CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[i] ,&statis[i], 4));                   // 读取状态字
        CHECK_FUNC(GM_RxPdoEntry_Write(OperationMode[i], 0x08));                        // 设置操作模式
    }
    debug("==============================握手前DMA写================================\n");
    rc = GM_Master_Send(1);
    CHECK_RC(rc, "握手前DMA写失败", err_close);

    debug("====================与主站握手:GM_Master_OP_Valid()=======================\n");
    rc = GM_Master_OP_Valid();
    CHECK_RC(rc, "与主站握手失败", err_close);

   sem_post(&op_sem); // 通知其他线程主站已经进入op

    while (!exit_flag){ // 进入控制逻辑循环
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
        post_rt_sem();
        // 电机控制逻辑
        if(!start_sem){
            for (int i = 0; i < SLAVENUM; i++)
            {
                CHECK_FUNC(GM_TxPdoEntry_Read(StatusWord[i], &statis[i] , 4));// 获取状态字

                if(!start_statis)
                {
                    CHECK_FUNC(GM_TxPdoEntry_Read(CurPos[i], &pos[i], 4));
                }
                // 根据状态字设置控制字
                if((statis[i] == 0x0270)){CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x06));}
                else if((statis[i] == 0x0231))
                {
                    CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x07));
                    CHECK_FUNC(GM_RxPdoEntry_Write(PurPos[i], pos[i]));
                }
                else if((statis[i] == 0x0233)){CHECK_FUNC(GM_RxPdoEntry_Write(ControlWord[i], 0x0f));}
                else if((statis[i] == 0x0237)||(statis[i] == 0x1237) )
                {
                    start_statis += 1;
                    if(start_statis > 100)
                    {
                        start_sem = true;
                    }
                }
            }
        }
        if(start_sem)
        {
            //hal read
            Hal_Driver_Update_All(true,false);






            //control logic
            Hal_Driver_Get_Actual_Pos(&x_axis_driver,&pos_act[0]);
            Hal_Driver_Get_Actual_Pos(&y_axis_driver,&pos_act[1]);
            Hal_Driver_Get_Actual_Pos(&z_axis_driver,&pos_act[2]);

            rc = HW_Get_Count(&current_count);
            rc = HW_Get_Axis_And_Rate(&axisandrate);
            rate = axisandrate & 0b111;
            rate = Get_Multiplier_Switch(rate);
            axis = (axisandrate>>3) & 0b11111;
            axis = Get_Axis_Switch(axis);
            if (rc < 0)
                goto close;
            if(axis == -1 || (last_axis != axis) || (last_rate != rate))
            {
                for(int i = 0; i < 5; ++i)
                {
                    simple_tp[i].first_pass = 1;
                }
                mpg_statis = true;
            }
            else if(axis >= 0 && axis < 5)
            {
                simple_tp[axis].joint.count = current_count;
                simple_tp[axis].joint.rate = rate;
                simple_tp[axis].joint.drivers_data = pos_act[axis];
                //假定加减速参数
                simple_tp[axis].joint.vel_limit = 500 * ratio;
                simple_tp[axis].joint.jaccel_limit = 2000 * ratio;
                //初始化
                if(mpg_statis)
                {
                    simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = pos_act[axis];
                    simple_tp[axis].curr_vel = 0;
                    //printf"----simple_tp[axis].joint.pos_cmd:%f %d\n",simple_tp[axis].joint.pos_cmd,axis);
                    mpg_statis = false;
                }

                Handlewheels_Motion(&simple_tp[axis]);
                Simple_Tp_Update(&simple_tp[axis]);
                simple_tp[axis].joint.pos_cmd = simple_tp[axis].curr_pos;
                ////printf"当前手轮计数：%u, 轴选：%d, 倍率：%d，手轮规划位置： %f  电机位置：%d\n", current_count, axis, rate,simple_tp[axis - 1].pos_cmd,pos_act);
            }
            if(!mpg_statis)
            {
                switch (axis)
                {
                case 0:
                    Hal_Driver_Set_Target_Pos(&x_axis_driver,simple_tp[0].joint.pos_cmd);
                    //printf"simple_tp[0].joint.pos_cmd:%f\n",simple_tp[0].joint.pos_cmd);
                    break;
                case 1:
                    Hal_Driver_Set_Target_Pos(&y_axis_driver,simple_tp[1].joint.pos_cmd);
                    //printf"simple_tp[1].joint.pos_cmd:%f\n",simple_tp[1].joint.pos_cmd);
                    break;
                case 2:
                    Hal_Driver_Set_Target_Pos(&z_axis_driver,simple_tp[2].joint.pos_cmd);
                    //printf"simple_tp[2].joint.pos_cmd:%f\n",simple_tp[2].joint.pos_cmd);
                    break;
                }
            }
            last_axis = axis;
            last_rate = rate;








            //hal write
            Hal_Driver_Update_All(false,true);
        }

        debug("=============================发送主站周期数据==============================\n");
        rc = GM_Master_Send(1);
        CHECK_RC(rc, "发送主站周期数据失败", err_close);
    }

    return NULL;
err_close:
    printf("pdo线程不正常退出");
    exit_flag = 1;
    return NULL;
close:
    // 释放IO
    IO_Resource_Release();
    return NULL;
}

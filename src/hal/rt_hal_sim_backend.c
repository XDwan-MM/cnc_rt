/* 调试模式下的驱动层桩（仿真后端）。
 *
 * 它扮演 GreeMaster 驱动层：对外提供与真实后端逐字相同的接口，但状态全在内存里推演——
 * 位置/速度即时跟随指令，状态字按控制字推导，故障与定向可以按需注入。这样 HAL core、
 * 适配层和上层业务时序都能在没有台架的情况下跑起来（离线回归靠的就是它）。
 *
 * 两条硬约束：
 *   1. 从站拓扑必须与 rt_hal_config.c 的 kBindings 一致，否则 HAL 在 start 阶段就报错；
 *   2. 它是全工程唯一包含 HAL 内部头的文件（桩需要 DeviceSlot / DevDictRole），
 *      别处不要照抄这个 include 列表。
 *
 * 可注入的故障与行为由环境变量控制，清单见下方各变量定义处的注释。 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdatomic.h>

#include "backend/greemaster/devices/device.h"
#include "backend/greemaster/devices/device_table.h"
#include "backend/greemaster/motion/servo_step.h"
#include "backend/greemaster/pdo/entry_access.h"
#include "backend/greemaster/bus/master.h"
#include "backend/greemaster/mapping/devdict.h"
#include "backend/greemaster/motion/ds402.h"
/* 定向控制字与完成位用本地 profile 的同一份定义，免得两处各写一份对不上 */
#include "rt_hal_config.h"

/* 从站拓扑，必须与 rt_hal_config.c 的 kBindings 一致，否则 HAL 在 start 阶段报错 */
#define SIM_SLAVE_PANEL   0
#define SIM_SLAVE_IO      1
#define SIM_SLAVE_SPINDLE 2
#define SIM_SLAVE_X       3
#define SIM_SLAVE_Y       4
#define SIM_SLAVE_Z       5
#define SIM_SLOT_COUNT    6

/* 模拟定向对准耗时（采样拍数）。真机由驱动器减速对准，几十到几百拍 */
#define SIM_ORIENT_DELAY  20

#define SIM_IO_ENTRIES    2u    /* IO 模块每方向 2 个 16 位 Entry = 32 位 */
#define SIM_PANEL_IN      11u   /* 面板输入 11 个 16 位 Entry */
#define SIM_PANEL_OUT     7u    /* 面板输出  7 个 16 位 Entry */

/* 单轴仿真状态，按槽号索引 */
typedef struct {
    uint32_t target_pos;
    uint32_t actual_pos;
    uint32_t target_speed;
    uint32_t actual_speed;
    uint16_t control_word;
    uint16_t previous_control_word;
    uint16_t op_mode;
    int      fault;         /* 非零 = 该轴处于 Fault 态，直到收到复位边沿 */
    int      orient_active;  /* 非零 = 正在定向（控制字是厂商那个） */
    int      orient_cycles;  /* 定向已经过的采样拍数；到 SIM_ORIENT_DELAY 置完成位 */
} SimServo;

/* 驱动层所有 HAL 依赖均由桩提供，避免静态链接混入真实主站对象 */
device_data_t g_device_data[MAX_DEVICE_NUM];

static DeviceSlot s_slots[SIM_SLOT_COUNT];
static DEVICE_BASIC_INFO s_identity[SIM_SLOT_COUNT];
static SimServo   s_servo[SIM_SLOT_COUNT];
static uint32_t   s_io_in[SIM_IO_ENTRIES];
static uint32_t   s_io_out[SIM_IO_ENTRIES];
static uint32_t   s_panel_in[SIM_PANEL_IN];
static uint32_t   s_panel_out[SIM_PANEL_OUT];
static uint32_t   s_cycle_us = 1000u;
static int        s_opened;
static int        s_slave_count = SIM_SLOT_COUNT;
static int        s_fail_bus;    /* 非零 = 所有驱动读都失败，用来验证总线故障路径 */
static int        s_csv_reached; /* CNC_RT_SIM_CSV_REACHED=1：上电即处于 CSV 模式并置"到速"位 bit10，
 * 用来验证换向与到速判定 */
/* 驱动层停止标志：Master_RequestStop() 置，周期里查 */
static atomic_int s_stop_requested;

/* 控制字 → 状态字，位模式同 CiA402 */
static uint16_t sim_status_from_ctrl(uint16_t cw) {
    if ((cw & 0x000Fu) == 0x000Fu) return 0x0237u;  /* 运行使能（Operation enabled）：真正出力 */
    if ((cw & 0x0007u) == 0x0007u) return 0x0023u;  /* 已合闸（Switched on）：功率级带电、不带载力矩 */
    if ((cw & 0x0006u) == 0x0006u) return 0x0021u;  /* 就绪（Ready to switch on）：等待合闸 */
    return 0x0040u;                                 /* 禁止合闸（Switch on disabled）：断电态 */
}

/* 位长必须与 HAL bind_axis/bind_io 期望一致，对不上返回 UNSUPPORTED */
static void set_bits(void* entry, uint8_t bits) {
    /* Tx/RxEntry_Unit 布局相同，头字段都是 index/subindex/bit_length */
    ((TxEntry_Unit*)entry)->bit_length = bits;
}

/* 建一张伺服槽；spindle 为真须补速度 PDO，HAL 要求主轴必须有速度 */
static void build_servo_slot(int idx, int slave, int axis, DEVICE_TYPE type, int spindle) {
    DeviceSlot* s = &s_slots[idx];
    memset(s, 0, sizeof(*s));
    s->type = type;
    s->slave_pos = slave;
    s->axis_index = axis;
    s->vendor_id = type == GREE_AXIS4_TYPE ? 2252u : 0x1234u;
    s->product_code = type == GREE_AXIS4_TYPE ? 2973639698u :
                      spindle ? 0x9001u : 0x9002u;
    s->revision = 1u;
    s->serial = (uint32_t)(1000 + slave);
    snprintf(s->name, sizeof(s->name), "%s", spindle ? "SIM spindle" : "SIM feed");

    slave_addr* h = &s->entries.slave;
    set_bits(&h->statusWord, 16);
    set_bits(&h->Control_word, 16);
    set_bits(&h->target_pos, 32);
    set_bits(&h->act_pos, 32);
    set_bits(&h->Modes_of_operation, 8);
    set_bits(&h->act_mode, 8);
    set_bits(&h->error_code, 16);
    if (spindle) {
        set_bits(&h->target_speed, 32);
        set_bits(&h->act_speed, 32);
    }
    /* 初值：未使能、位置与速度都是 0、未故障 */
    s_servo[idx].control_word = 0;
    s_servo[idx].fault = 0;
}

static void build_panel_slot(int idx, int slave) {
    DeviceSlot* s = &s_slots[idx];
    memset(s, 0, sizeof(*s));
    s->type = CONTROL_PANEL_TYPE;
    s->slave_pos = slave;
    s->axis_index = -1;
    snprintf(s->name, sizeof(s->name), "SIM panel");
    s->entries.control.in_count = (uint8_t)SIM_PANEL_IN;
    s->entries.control.out_count = (uint8_t)SIM_PANEL_OUT;
    for (unsigned i = 0; i < SIM_PANEL_IN; ++i) set_bits(&s->entries.control.input_addr[i], 16);
    for (unsigned i = 0; i < SIM_PANEL_OUT; ++i) set_bits(&s->entries.control.output_addr[i], 16);
}

static void build_io_slot(int idx, int slave) {
    DeviceSlot* s = &s_slots[idx];
    memset(s, 0, sizeof(*s));
    s->type = IO_MODEL_TYPE;
    s->slave_pos = slave;
    s->axis_index = -1;
    snprintf(s->name, sizeof(s->name), "SIM io");
    s->entries.io.io_in_count = (uint8_t)SIM_IO_ENTRIES;
    s->entries.io.io_out_count = (uint8_t)SIM_IO_ENTRIES;
    for (unsigned i = 0; i < SIM_IO_ENTRIES; ++i) {
        set_bits(&s->entries.io.io_input_addr[i], 16);
        set_bits(&s->entries.io.io_output_addr[i], 16);
    }
}

/* 驱动层接口（签名须与后端头文件逐字一致，否则链接器会拉进真实现，
 * 变成"一半真一半假"） */

/* CNC_RT_SIM_FAULT_STUCK=1：注入的 Fault 不因 bit7 边沿消失，把复位一直按在 PENDING
 * 上（复位在飞窗口拉成 5 秒），好验"急停优先于复位"。 */
static int s_fault_stuck;
/* CNC_RT_SIM_ERR=<从站>[,<码>]：该轴报出驱动器故障码 0x603F（默认 0x1234） */
static uint16_t s_err_code[SIM_SLOT_COUNT];

int ethercat_init(const MasterConfig* cfg) {
    if (!cfg) return -1;
    const char* reached = getenv("CNC_RT_SIM_CSV_REACHED");
    s_csv_reached = reached && atoi(reached);
    memset(s_servo, 0, sizeof(s_servo));
    memset(s_io_in, 0, sizeof(s_io_in));
    memset(s_io_out, 0, sizeof(s_io_out));
    memset(s_panel_in, 0, sizeof(s_panel_in));
    memset(s_panel_out, 0, sizeof(s_panel_out));
    memset(s_err_code, 0, sizeof(s_err_code));

    /* CNC_RT_SIM_FAULT=<从站号>：该轴上电即处于 Fault */
    const char* fault_slave = getenv("CNC_RT_SIM_FAULT");
    const char* gree4 = getenv("CNC_RT_SIM_GREE4");
    const int four_in_one = gree4 && atoi(gree4);

    build_panel_slot(0, SIM_SLAVE_PANEL);
    build_io_slot(1, SIM_SLAVE_IO);
    s_slave_count = four_in_one ? 3 : SIM_SLOT_COUNT;
    build_servo_slot(2, SIM_SLAVE_SPINDLE, 0,
                     four_in_one ? GREE_AXIS4_TYPE : SERVO_TYPE, 1);
    if (s_csv_reached) s_servo[2].op_mode = DS402_MODE_CSV;
    build_servo_slot(3, four_in_one ? SIM_SLAVE_SPINDLE : SIM_SLAVE_X,
                     four_in_one ? 1 : 0,
                     four_in_one ? GREE_AXIS4_TYPE : SERVO_TYPE, 0);
    build_servo_slot(4, four_in_one ? SIM_SLAVE_SPINDLE : SIM_SLAVE_Y,
                     four_in_one ? 2 : 0,
                     four_in_one ? GREE_AXIS4_TYPE : SERVO_TYPE, 0);
    build_servo_slot(5, four_in_one ? SIM_SLAVE_SPINDLE : SIM_SLAVE_Z,
                     four_in_one ? 3 : 0,
                     four_in_one ? GREE_AXIS4_TYPE : SERVO_TYPE, 0);
    memset(s_identity, 0, sizeof(s_identity));
    for (int i = 0; i < SIM_SLOT_COUNT; ++i) {
        DEVICE_BASIC_INFO* id = &s_identity[s_slots[i].slave_pos];
        id->ID = s_slots[i].vendor_id;
        id->CODE = s_slots[i].product_code;
        id->Revision = s_slots[i].revision;
        id->Serial = s_slots[i].serial;
        id->type = s_slots[i].type;
    }

    /* 全零面板输入等于"急停一直按着"、会拒掉所有复位事件，故默认置"未按下"；
     * 测按下用 CNC_RT_SIM_ESTOP=1。 */
    s_panel_in[2] = 0x0010u;   /* bit4 = X8.4 = 急停，低有效 → 1 表示未按下 */
    {
        const char* estop = getenv("CNC_RT_SIM_ESTOP");
        if (estop && *estop && atoi(estop)) {
            s_panel_in[2] = 0;
            fprintf(stderr, "[SIM] 注入急停按下（CNC_RT_SIM_ESTOP）\n");
        }
    }

    if (fault_slave && *fault_slave) {
        const int want = atoi(fault_slave);
        for (int i = 0; i < SIM_SLOT_COUNT; ++i)
            if (s_slots[i].slave_pos == want) {
                s_servo[i].fault = 1;
                fprintf(stderr, "[SIM] 从站 %d 注入 Fault 态（CNC_RT_SIM_FAULT）\n", want);
            }
    }
    {
        const char* stuck = getenv("CNC_RT_SIM_FAULT_STUCK");
        s_fault_stuck = (stuck && *stuck && atoi(stuck)) ? 1 : 0;
        if (s_fault_stuck)
            fprintf(stderr, "[SIM] Fault 不因复位边沿消失（CNC_RT_SIM_FAULT_STUCK）\n");
    }
    {
        /* 注入驱动器故障码 0x603F，码默认 0x1234 */
        const char* e = getenv("CNC_RT_SIM_ERR");
        if (e && *e) {
            unsigned long code = 0x1234u;
            const char* comma = strchr(e, ',');
            const int want = atoi(e);
            if (comma) code = strtoul(comma + 1, NULL, 0);
            for (int i = 0; i < SIM_SLOT_COUNT; ++i)
                if (s_slots[i].slave_pos == want) {
                    s_err_code[i] = (uint16_t)code;
                    fprintf(stderr, "[SIM] 从站 %d 注入驱动器故障码 0x%04X（CNC_RT_SIM_ERR）\n",
                            want, (unsigned)s_err_code[i]);
                }
        }
    }

    /* CNC_RT_SIM_FAIL=1：所有驱动读失败 */
    {
        const char* f = getenv("CNC_RT_SIM_FAIL");
        s_fail_bus = (f && *f && atoi(f)) ? 1 : 0;
        if (s_fail_bus) fprintf(stderr, "[SIM] 注入总线故障（CNC_RT_SIM_FAIL）\n");
    }

    s_cycle_us = cfg->cycle_us ? cfg->cycle_us : 1000u;
    s_stop_requested = 0;    
    s_opened = 1;
    return 0;
}

int ethercat_close(void) {
    s_opened = 0;
    return 0;
}

int Master_SlaveCount(void) { return s_opened ? s_slave_count : 0; }

const char* Master_StartupError(void) { return ""; }

const DEVICE_BASIC_INFO* device_identity_get(int pos) {
    static const DEVICE_BASIC_INFO empty;
    return s_opened && pos >= 0 && pos < s_slave_count ? &s_identity[pos] : &empty;
}

void Master_RequestStop(void) { atomic_store(&s_stop_requested, 1); }
int Master_StopFlag(void) { return !s_opened || atomic_load(&s_stop_requested); }

int DeviceTable_Get(const DeviceSlot** out) {
    if (out) *out = s_opened ? s_slots : NULL;
    return s_opened ? SIM_SLOT_COUNT : 0;
}

int Master_WaitCycle(void) {
    if (!s_opened) return -1;
    if (atomic_load(&s_stop_requested)) return MASTER_STOP_REQUESTED;
    /* 无真实中断，用睡眠代替节拍 */
    struct timespec ts;
    ts.tv_sec = (time_t)(s_cycle_us / 1000000u);
    ts.tv_nsec = (long)(s_cycle_us % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
    return atomic_load(&s_stop_requested) ? MASTER_STOP_REQUESTED : 0;
}

int Master_CommitCycle(void) {
    if (!s_opened) return -1;
    if (atomic_load(&s_stop_requested)) return MASTER_STOP_REQUESTED;
    /* CNC_RT_SIM_CYCLE_FAIL=<第几拍>：到那一拍返回失败 */
    {
        static long fail_at = -1;
        static long commits;
        if (fail_at < 0) {
            const char* f = getenv("CNC_RT_SIM_CYCLE_FAIL");
            fail_at = f ? strtol(f, NULL, 10) : 0;
        }
        if (++commits == fail_at) {
            fprintf(stderr, "[SIM] 注入周期错误（CNC_RT_SIM_CYCLE_FAIL=%ld）\n", fail_at);
            return -1;
        }
    }
    /* 只有 bit7 上升沿清 Fault */
    for (int i = 0; i < SIM_SLOT_COUNT; ++i) {
        SimServo* v = &s_servo[i];
        if (!s_fault_stuck && v->fault && (v->control_word & 0x0080u) &&
            !(v->previous_control_word & 0x0080u)) v->fault = 0;
        v->previous_control_word = v->control_word;
    }
    return 0;
}

int Master_BusHealth(MasterBusHealth* out) {
    /* 仿真总线：无警告、无 CRC/超时计数；要验证报警链路就在这里造数 */
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    return 0;
}

int Master_ServoRead(int slot, DevDictRole role, uint32_t* out) {
    if (s_fail_bus) return -1;
    if (!out || slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    SimServo* v = &s_servo[slot];   /* 定向状态要在这里推进，不能再是 const */
    switch (role) {
    /* Fault 优先于按控制字推出的状态（真实驱动器故障时状态字 bit3 置位） */
    case DEV_DICT_ROLE_STATUS_WORD:
        if (v->fault) { *out = 0x0008u; return 0; }
        /* 厂商定向字不是标准 DS402 控制字：低 4 位同样是使能，bit10 是厂商"开始定向"；
         * sim_status_from_ctrl 认不出，故单独判。 */
        if (v->orient_active) {
            const uint16_t base = sim_status_from_ctrl(RT_PLACEHOLDER_ORIENT_CW);
            ++v->orient_cycles;
            *out = v->orient_cycles >= SIM_ORIENT_DELAY
                       ? (uint16_t)(base | RT_PLACEHOLDER_ORIENT_DONE)
                       : base;
            return 0;
        }
        *out = sim_status_from_ctrl(v->control_word);
        /* CSV 下 bit10 = 速度到达（含目标为 0 的静止态） */
        if (s_csv_reached && v->op_mode == DS402_MODE_CSV &&
            v->actual_speed == v->target_speed)
            *out |= RT_PLACEHOLDER_ORIENT_DONE;
        return 0;
    /* 位置与速度即时跟随指令：写什么读回什么 */
    case DEV_DICT_ROLE_ACTUAL_POS:    *out = v->actual_pos; return 0;
    case DEV_DICT_ROLE_ACTUAL_SPEED:  *out = v->actual_speed; return 0;
    case DEV_DICT_ROLE_ACTUAL_TORQUE: *out = 0; return 0;
    case DEV_DICT_ROLE_MODE_DISPLAY:  *out = v->op_mode; return 0;
    case DEV_DICT_ROLE_ERROR_CODE:    *out = s_err_code[slot]; return 0;
    default:                          *out = 0; return 0;
    }
}

int Master_ServoWrite(int slot, DevDictRole role, uint32_t value) {
    if (slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    SimServo* v = &s_servo[slot];
    switch (role) {
    case DEV_DICT_ROLE_TARGET_POS:
        v->target_pos = value;
        v->actual_pos = value;      /* 仿真：位置即时到位 */
        return 0;
    case DEV_DICT_ROLE_TARGET_SPEED:
        v->target_speed = value;
        v->actual_speed = value;    /* 仿真：转速即时到位 */
        return 0;
    case DEV_DICT_ROLE_CONTROL_WORD:
        /* 进入/退出定向要重置计时：否则第二次 M19 一上来就报 DONE（真机须重新对准） */
        if ((uint16_t)value != v->control_word) {
            v->orient_active = ((uint16_t)value == RT_PLACEHOLDER_ORIENT_CW);
            v->orient_cycles = 0;
        }
        v->control_word = (uint16_t)value;
        return 0;
    case DEV_DICT_ROLE_OP_MODE:
        v->op_mode = (uint16_t)value;
        return 0;
    default:
        return 0;                   /* 转矩等未建模的角色：收下不做事 */
    }
}

int Master_SdoTransfer(int slave_pos, uint16_t index, uint8_t subindex,
                       void* data, uint16_t size, int write, uint32_t timeout_ms) {
    (void)timeout_ms;
    if (!s_opened || atomic_load(&s_stop_requested)) return -4;
    if (!data || subindex != 0 || (size != 1 && size != 2 && size != 4)) return -3;

    int slot = -1;
    for (int i = 0; i < SIM_SLOT_COUNT; ++i)
        if (s_slots[i].slave_pos == slave_pos && s_slots[i].type == SERVO_TYPE) {
            slot = i;
            break;
        }
    if (slot < 0) return -3;

    DevDictRole role;
    uint16_t expected_size;
    switch (index) {
    case 0x6040: role = DEV_DICT_ROLE_CONTROL_WORD; expected_size = 2; break;
    case 0x6041: role = DEV_DICT_ROLE_STATUS_WORD; expected_size = 2; break;
    case 0x603F: role = DEV_DICT_ROLE_ERROR_CODE; expected_size = 2; break;
    case 0x6060: role = DEV_DICT_ROLE_OP_MODE; expected_size = 1; break;
    case 0x6061: role = DEV_DICT_ROLE_MODE_DISPLAY; expected_size = 1; break;
    case 0x6064: role = DEV_DICT_ROLE_ACTUAL_POS; expected_size = 4; break;
    case 0x606C: role = DEV_DICT_ROLE_ACTUAL_SPEED; expected_size = 4; break;
    case 0x6077: role = DEV_DICT_ROLE_ACTUAL_TORQUE; expected_size = 2; break;
    case 0x607A: role = DEV_DICT_ROLE_TARGET_POS; expected_size = 4; break;
    case 0x60FF: role = DEV_DICT_ROLE_TARGET_SPEED; expected_size = 4; break;
    default: return -3;
    }
    if (size != expected_size) return -3;
    if (write) {
        if (index != 0x6040 && index != 0x6060 && index != 0x607A && index != 0x60FF)
            return -3;
        uint32_t value = 0;
        memcpy(&value, data, size);
        return Master_ServoWrite(slot, role, value) ? -3 : 0;
    }
    if (index == 0x6040 || index == 0x607A || index == 0x60FF) return -3;
    uint32_t value = 0;
    if (Master_ServoRead(slot, role, &value)) return -3;
    memcpy(data, &value, size);
    return 0;
}

/* 与真实 servo_step.c 同构：预置目标 → 改模式 → 写控制字，顺序不能变；漏掉写控制字
 * 会让 DS402 状态机空转（使能/去使能/故障复位都推不动）。 */
 /* HAL 会按驱动器身份给每槽下发模式切换策略（台达 DISABLE_THEN_MODE）；调试后端跟随记录。 */
static Ds402ModeSwitch s_sim_msw[SIM_SLOT_COUNT];

int Master_ServoSetModeSwitch(int slot, Ds402ModeSwitch how) {
    if (slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    if (how != DS402_MODESW_DISABLE_FIRST && how != DS402_MODESW_IN_PLACE &&
        how != DS402_MODESW_DISABLE_THEN_MODE) return -1;
    s_sim_msw[slot] = how;
    return 0;
}
int Master_ServoStep(int slot, Ds402Request req, uint16_t mode, uint16_t* out) {
    if (slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    SimServo* v = &s_servo[slot];

    const uint16_t sw = v->fault ? 0x0008u : sim_status_from_ctrl(v->control_word);
    if (out) *out = sw;

    const Ds402Step step = Ds402_NextStepReq(sw, v->op_mode, req, mode, DS402_MODESW_DISABLE_FIRST);
    if (step.preset_target) v->target_pos = v->actual_pos;
    if (step.op_mode != 0xFFFFu) v->op_mode = step.op_mode;
    if (step.kind != DS402_STEP_NONE && step.kind != DS402_STEP_DONE &&
        step.kind != DS402_STEP_SWITCH_MODE)
        v->control_word = step.control_word;
    return 0;
}

int Master_IoReadEntry(int slot, int index, uint32_t* value, int* bit_length) {
    if (!value || !bit_length || slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    if (s_slots[slot].type == CONTROL_PANEL_TYPE) {
        if (index < 0 || (unsigned)index >= SIM_PANEL_IN) return -1;
        *value = s_panel_in[index];
    } else {
        if (index < 0 || (unsigned)index >= SIM_IO_ENTRIES) return -1;
        *value = s_io_in[index];
    }
    *bit_length = 16;
    return 0;
}

int Master_IoWriteEntry(int slot, int index, uint32_t value) {
    if (slot < 0 || slot >= SIM_SLOT_COUNT) return -1;
    if (s_slots[slot].type == CONTROL_PANEL_TYPE) {
        if (index < 0 || (unsigned)index >= SIM_PANEL_OUT) return -1;
        s_panel_out[index] = value;
    } else {
        if (index < 0 || (unsigned)index >= SIM_IO_ENTRIES) return -1;
        s_io_out[index] = value;
    }
    return 0;
}

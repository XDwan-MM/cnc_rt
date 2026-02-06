#ifndef LADDATA_H
#define LADDATA_H

#include "regdata.h"
#include <cstdint>

#define LAD_CELL_PER_ROW 10   // 梯图中一行元件个数
#define LAD_IS_NUM 100        // 指令集数量
#define LAD_MAX_LOOP_LEVEL 6  // 循环最大层数
#define LAD_MAX_EDGE_NUM 1024 // 上升沿下降沿最大数目

#define LAD_ROW_MAX_NUM 5000     // 梯形图行最大数量
#define LAD_IL_MAX_NUM 10000     // 指令表最大数量
#define LAD_SUBPRG_MAX_NUM 128   // 子程序最大数量
#define LAD_SYMBL_MAX_NUM 5000   // 符号表最大数量
#define LAD_REG_MAX_NUM 24000    // 寄存器信息
#define LAD_IO_MAX_NUM (256 * 8) // IO对照表最大数量
#define LAD_REG_LOCK_MAX_NUM 32	 // 锁定寄存器表最大数量

#define LAD_USER_IQ_NUM 80

#define LAD_SYMBOL_NAME_LEN 16
#define LAD_SYMBOL_COMMENTS_LEN 76

#define LAD_PRG_NAME_LEN 16
#define LAD_PRG_EDITOR_LEN 32
#define LAD_PRG_COMMENTS_LEN 64

typedef struct LadReg {
    RegType type; // 寄存器类型 x5.7 中的 'x'
    int idx;      // 寄存器下标 x5.7 中的5
    char bit;     // 寄存器位， x5.7 中的 '7' , -1表示无效
} LadReg;

typedef enum Opcode {
    OP_NONE,
    OP_VLINE,
    OP_HLINE,
    OP_LD,
    OP_LDI,
    OP_OUT,
    OP_OOUT,
    OP_SET,
    OP_RST,
    OP_AND,
    OP_ANI,
    OP_OR,
    OP_ORI,
    OP_LDP,
    OP_LDF,
    OP_ANDP,
    OP_ANDF,
    OP_ORP,
    OP_ORF,
    OP_ORB,
    OP_ANB,
    OP_MPS,
    OP_MRD,
    OP_MPP,
    OP_IEND,
    OP_1END,
    OP_2END,
    OP_SP,
    OP_SPE,
    OP_ADD,
    OP_SUB,
    OP_MUL,
    OP_DIV,
    OP_INC,
    OP_DEC,
    OP_WAND,
    OP_WOR,
    OP_WXOR,
    OP_SHL,
    OP_SHR,
    OP_ROL,
    OP_ROR,
    OP_MOV,
    OP_TMRB,
    OP_STMR,
    OP_CTRC,
    OP_JMP,
    OP_LBL,
    OP_CALL,
    OP_TOTAL
} Opcode;

// 函数块类型
typedef struct FuncRectType {
    bool hasLeft;
    bool hasRight;
    bool hasTop;
    bool hasBottom;
} FuncRectType;

typedef struct LadCell {
    bool hasVLine;         // 是否有垂直线
    bool hasParam;         // 单元格是否能输入参数
    int instrIdx;          // 在指令表中的位置
    int widthPos;          // 本单元格相对元件原点的相对宽度位置
    int heightPos;         // 本单元格相对元件原点的相对高度位置
    Opcode opcode;         // 命令Id
    FuncRectType rectType; // 函数块类型
    LadReg reg;            // 单元格中的寄存器
} LadCell;

typedef struct LadRow {
    LadCell cells[LAD_CELL_PER_ROW]; // 梯形图单元格数组
} LadRow;

typedef struct LadSymbol {
    LadReg reg;                             // 寄存器
    char name[LAD_SYMBOL_NAME_LEN];         // 符号名称
    char comments[LAD_SYMBOL_COMMENTS_LEN]; // 符号注释
} LadSymbol;

typedef struct LadPrgInfo {
    char name[LAD_PRG_NAME_LEN];     // 程序名
    char editor[LAD_PRG_EDITOR_LEN]; // 编辑者
    int version;                     // plc内部版本号
    int rowCount;                    // 梯图行数
    int subprgCount;                 // 子程序数
    int symbolCount;                 // 符号表数
    int instrCount;                  // 梯图步数（指令表数）
    int ioCount;                     // IO对照表数
    int userIoCount;                 // USR对照表数
    int mcpIoCount;                  // MCP对照表数
    int regLockCount;                // 锁定寄存器表数
    char machineName[16];            // 机床名称
    char facturer[32];               // 生产厂家
    char comment[256];               // 注释
    int regCount;                    // 梯形图操作数数量
    bool isRunning;                  // 是否启动
    int64_t createTime;              // 文件创建时间
    int64_t modifyTime;              // 文件最后修改时间
    int64_t plc2CurCycle;            // PLC2当前周期
    int64_t plc2MinCycle;            // PLC2最小周期
    int64_t plc2MaxCycle;            // PLC2最大周期
} LadPrgInfo;

typedef struct LadSubprgInfo {
    int no;
    char name[LAD_PRG_NAME_LEN];         // 梯形图程序名称
    char editor[LAD_PRG_EDITOR_LEN];     // 梯形图程序作者
    char comments[LAD_PRG_COMMENTS_LEN]; // 梯形图程序作者
    int rowCount;                        // 梯图行数
    int instrCount;                      // 梯图步数（指令表数）
    int beginRow;                        // 子程序在梯图中的开始行号
} LadSubprgInfo;

typedef enum LadForceState {
    LAD_FORCE_NORMAL,  // 正常
    LAD_FORCE_ENABLE,  // 允许
    LAD_FORCE_DISABLE, // 禁止
} LadForceState;

typedef struct LadInstr {
    Opcode opcode;       // 操作符
    int regIdx;          // 操作数对应的 index
    LadForceState state; // 强制状态
} LadInstr;

typedef struct LadIoMapping {
    LadReg src; // 寄存器(X/Y)
    LadReg dir; // 寄存器(I/Q)
    char flag;  // 0为高电平,非0为低电平
    char type;  // 0-plc1, 非0-plc2
} LadIoMapping;

typedef struct LadRegLock {
    LadReg reg;    // 锁定寄存器
    bool lockflag; // 锁定标记，0：解锁，非0：锁定。
    bool newflag;  // 写入标记，0：未写入，非0：写入。
    char format;   // 显示格式，0：二进制，1：十进制，2：十六进制
    int newval;    // 设置值
} LadRegLock;

typedef enum LadTimerType {
    T_TYPE_NONE,
    T_TYPE_TMRB,
    T_TYPE_STMR
} LadTimerType;

typedef enum LadCounterType {
    C_TYPE_NONE,
    C_TYPE_CTRC,
} LadCounterType;

typedef struct LadData {
    LadRow rows[LAD_ROW_MAX_NUM];                  // 梯形图行
    LadSymbol symbols[LAD_SYMBL_MAX_NUM];          // 梯形图符号表
    LadPrgInfo info;                               // 梯形图信息
    LadSubprgInfo subprgInfos[LAD_SUBPRG_MAX_NUM]; // 梯形图子程序信息
    LadInstr instrs[LAD_IL_MAX_NUM];               // 梯形图指令表数组
    LadReg regs[LAD_REG_MAX_NUM];                  // 梯形图操作数数组
    LadIoMapping ioMappings[LAD_IO_MAX_NUM];       // 梯形图IO对照表
    LadRegLock regLocks[LAD_REG_LOCK_MAX_NUM];     // 梯形图锁定寄存器表
    LadTimerType timerTypes[REG_T_NUM];            // 梯形图定时器类型
    unsigned int timerPreset[REG_T_NUM];           // 定时器预设值
    LadCounterType counterTypes[REG_C_NUM];        // 梯形图计数器类型
    unsigned int counterPreset[REG_C_NUM];         // 计数器预设值
} LadData;

#endif
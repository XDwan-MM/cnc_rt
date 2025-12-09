#ifndef LADDATA_H
#define LADDATA_H

#include "regdata.h"

#define LAD_CELL_PER_ROW 10   // 梯图中一行元件个数
#define LAD_IS_NUM 100        // 指令集数量
#define LAD_MAX_LOOP_LEVEL 6  // 循环最大层数
#define LAD_MAX_EDGE_NUM 1024 // 上升沿下降沿最大数目

#define LAD_ROW_MAX_NUM 5000   // 梯形图行最大数量
#define LAD_IL_MAX_NUM 10000   // 指令表最大数量
#define LAD_SUBPRG_MAX_NUM 128 // 子程序最大数量
#define LAD_SYMBL_MAX_NUM 5000 // 符号表最大数量

#define LAD_SYMBOL_NAME_LEN 16
#define LAD_SYMBOL_COMMENTS_LEN 76

#define LAD_PRG_NAME_LEN 64
#define LAD_PRG_EDITOR_LEN 32

struct LadReg {
    RegType type;
    int idx;
    char bit;
};

enum Opcode {
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
    OP_ANC,
    OP_ANNC,
    OP_OR,
    OP_ORI,
    OP_ORC,
    OP_ORNC,
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
    OP_2END
};

// 函数块类型
struct FuncRectType {
    bool hasLeft;
    bool hasRight;
    bool hasTop;
    bool hasBottom;
};

struct LadCell {
    bool hasVLine;
    bool hasParam;
    int instrIdx;
    Opcode opcode;
    FuncRectType rectType;
    LadReg reg;
};

struct LadRow {
    LadCell cells[LAD_CELL_PER_ROW];
};

struct LadSymbol {
    LadReg reg;
    char name[LAD_SYMBOL_NAME_LEN];
    char comments[LAD_SYMBOL_COMMENTS_LEN];
};

struct LadPrgInfo {
    char name[LAD_PRG_NAME_LEN];
    char editor[LAD_PRG_EDITOR_LEN];
    int rowCount;
    int subprgCount;
    int symbolCount;
    int instrCount;
};

struct LadSubprgInfo {
    int id;
    char name[LAD_PRG_NAME_LEN];
    char editor[LAD_PRG_EDITOR_LEN];
    int rowCount;
    int instrCount;
    int beginRow;
};

struct LadData {
    LadRow rows[LAD_ROW_MAX_NUM];
    LadSymbol symbols[LAD_SYMBL_MAX_NUM];
    LadPrgInfo info;
    LadSubprgInfo subprgInfos[LAD_SUBPRG_MAX_NUM];
};

#endif

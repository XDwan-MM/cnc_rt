#ifndef REGDATA_H
#define REGDATA_H

#define REG_R_NUM 2024
#define REG_W_NUM 256
#define REG_D_NUM 256
#define REG_X_NUM 512
#define REG_Y_NUM 512
#define REG_F_NUM 512
#define REG_G_NUM 512
#define REG_I_NUM 128
#define REG_Q_NUM 128
#define REG_B_NUM 512
#define REG_K_NUM 256
#define REG_T_NUM 512
#define REG_C_NUM 512

enum RegType {
    REG_R = 'R',
    REG_W = 'W',
    REG_D = 'D',
    REG_X = 'X',
    REG_Y = 'Y',
    REG_F = 'F',
    REG_G = 'G',
    REG_I = 'I',
    REG_Q = 'Q',
    REG_B = 'B',
    REG_K = 'K',
    REG_T = 'T',
    REG_C = 'C'
};

typedef struct {
    int current; // 当前值
    int preset;  // 预设值
} RegTimerCounter;

struct RegData {
    char regR[REG_R_NUM];
    short regW[REG_W_NUM];
    int regD[REG_D_NUM];
    char regX[REG_X_NUM];
    char regY[REG_Y_NUM];
    char regF[REG_F_NUM];
    char regG[REG_G_NUM];
    char regI[REG_I_NUM];
    char regQ[REG_Q_NUM];
    int regB[REG_B_NUM];
    char regK[REG_K_NUM];
    RegTimerCounter regT[REG_T_NUM];
    RegTimerCounter regC[REG_C_NUM];
};

#endif

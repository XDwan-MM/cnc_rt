#ifndef REGDATA_H
#define REGDATA_H

#define REG_A_NUM 128
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
#define REG_S_NUM 128

typedef enum {
    REG_TYPE_NONE = 0,
    REG_A = 'A',
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
    REG_C = 'C',
    REG_S = 'S',
    REG_L = 'L'
}RegType;

typedef struct {
    int current; // 当前值
    int preset;  // 预设值
} RegTimerCounter;

struct RegData {
    unsigned char regA[REG_A_NUM];
    unsigned char regR[REG_R_NUM];
    unsigned short regW[REG_W_NUM];
    unsigned int regD[REG_D_NUM];
    unsigned char regX[REG_X_NUM];
    unsigned char regY[REG_Y_NUM];
    unsigned char regF[REG_F_NUM];
    unsigned char regG[REG_G_NUM];
    unsigned char regI[REG_I_NUM];
    unsigned char regQ[REG_Q_NUM];
    unsigned int regB[REG_B_NUM];
    unsigned char regK[REG_K_NUM];
    RegTimerCounter regT[REG_T_NUM];
    RegTimerCounter regC[REG_C_NUM];
};

#endif

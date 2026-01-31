#include "cyclescanning.h"

int32 TM[100];
int32 plcRunNum;
int16 WW[REG_W_NUM];
int32 PP[REG_P_NUM], BB[REG_B_NUM], DD[REG_D_NUM];
uint8 XX[REG_X_NUM], YY[REG_Y_NUM], II[REG_I_NUM], QQ[REG_Q_NUM], RR[REG_R_NUM], FF[REG_F_NUM], GG[REG_G_NUM] /*,AA[REG_A_NUM]*/;

static LadInstr PLCRun[10000];
static LadReg PlcPara[24000];
RegData CurrentRegData = {0}, LastRegData = {0};

typedef void (*tFunc)(int NO);
tFunc array[512];

static int32 ST, MC, MP, sp_inn_pos, run_row, FOR_pos, For_time, init_end, plc1_end, plc2_end;
static int32 SP_inn[16];
bool init_system = 0;

typedef struct {
    char reg_type;
    int index;
    char bit;
    char last;
    char now;
} typePNEdge;

QList<typePNEdge> m_PN1Buf, m_PN2Buf, *m_PNBuf;

long GetReg(char reg, int index) {
    long n = 0;
    char a;
    short int b;
    switch (reg) {
        case 0:
            n = index;
            break;
        case 'x':
        case 'X':
            a = _gnc_get_x_value(index);
            n = (long)a;
            break;

        case 'y':
        case 'Y':
            a = _gnc_get_y_value(index);
            n = (long)a;
            break;

        case 'f':
        case 'F':
            a = _gnc_get_f_value(index);
            n = (long)a;
            break;

        case 'g':
        case 'G':
            a = _gnc_get_g_value(index);
            n = (long)a;
            break;

        case 'r':
        case 'R':
            a = _gnc_get_r_value(index);
            n = (long)a;
            break;

        case 'w':
        case 'W':
            b = _gnc_get_w_value(index);
            n = (long)b;
            break;

        case 'd':
        case 'D':
            b = _gnc_get_d_value(index);
            n = (long)b;
            break;

        case 'p':
        case 'P':
            n = _gnc_get_p_value(index);
            break;

        case 'b':
        case 'B':
            n = _gnc_get_b_value(index);
            break;

        case 't':
        case 'T':
            n = TM[index];
            break;
    }
    return n;
}

void SetReg(char reg, int index, long value) {
    char a;
    short int b;
    switch (reg) {
        case 'x':
        case 'X':
            a = (char)value;
            _gnc_set_x_value(index, a);
            break;
        case 'y':
        case 'Y':
            a = (char)value;
            _gnc_set_y_value(index, a);
            break;
        case 'f':
        case 'F':
            a = (char)value;
            _gnc_set_f_value(index, a);
            break;
        case 'g':
        case 'G':
            a = (char)value;
            _gnc_set_g_value(index, a);
            break;
        case 'r':
        case 'R':
            a = (char)value;
            _gnc_set_r_value(index, a);
            break;
        case 'w':
        case 'W':
            b = (short int)value;
            _gnc_set_w_value(index, b);
            break;
        case 'd':
        case 'D':
            b = (short int)value;
            _gnc_set_d_value(index, b);
            break;
        case 'p':
        case 'P':
            _gnc_set_p_value(index, value);
            break;
        case 'b':
        case 'B':
            _gnc_set_b_value(index, value);
            break;
        case 't':
        case 'T':
            TM[index] = value;
            break;
    }
}

void SetRegBit(char reg, int index, int bit) {
    switch (reg) {
        case 'x':
        case 'X':
            _gnc_set_x_bit(index, bit);
            break;
        case 'y':
        case 'Y':
            _gnc_set_y_bit(index, bit);
            break;
        case 'f':
        case 'F':
            _gnc_set_f_bit(index, bit);
            break;
        case 'g':
        case 'G':
            _gnc_set_g_bit(index, bit);
            break;
        case 'r':
        case 'R':
            _gnc_set_r_bit(index, bit);
            break;
        case 'w':
        case 'W':
            _gnc_set_w_bit(index, bit);
            break;
        case 'd':
        case 'D':
            _gnc_set_d_bit(index, bit);
            break;
        case 'p':
        case 'P':
            _gnc_set_p_bit(index, bit);
            break;
        case 'b':
        case 'B':
            _gnc_set_b_bit(index, bit);
            break;
    }
}

bool GetRegBit(char reg, int index, int bit) {
    switch (reg) {
        case 'x':
        case 'X':
            if (_gnc_get_x_bit(index, bit) != 0) return true;
            break;
        case 'y':
        case 'Y':
            if (_gnc_get_y_bit(index, bit) != 0) return true;
            break;
        case 'f':
        case 'F':
            if (_gnc_get_f_bit(index, bit) != 0) return true;
            break;
        case 'g':
        case 'G':
            if (_gnc_get_g_bit(index, bit) != 0) return true;
            break;
        case 'r':
        case 'R':
            if (_gnc_get_r_bit(index, bit) != 0) return true;
            break;
        case 'w':
        case 'W':
            if (_gnc_get_w_bit(index, bit) != 0) return true;
            break;
        case 'd':
        case 'D':
            if (_gnc_get_d_bit(index, bit) != 0) return true;
            break;
        case 'p':
        case 'P':
            if (_gnc_get_p_bit(index, bit) != 0) return true;
            break;
        case 'b':
        case 'B':
            if (_gnc_get_b_bit(index, bit) != 0) return true;
            break;
    }
    return false;
}

void ClrRegBit(char reg, int index, int bit) {
    switch (reg) {
        case 'x':
        case 'X':
            _gnc_clr_x_bit(index, bit);
            break;
        case 'y':
        case 'Y':
            _gnc_clr_y_bit(index, bit);
            break;
        case 'f':
        case 'F':
            _gnc_clr_f_bit(index, bit);
            break;
        case 'g':
        case 'G':
            _gnc_clr_g_bit(index, bit);
            break;
        case 'r':
        case 'R':
            _gnc_clr_r_bit(index, bit);
            break;
        case 'w':
        case 'W':
            _gnc_clr_w_bit(index, bit);
            break;
        case 'd':
        case 'D':
            _gnc_clr_d_bit(index, bit);
            break;
        case 'p':
        case 'P':
            _gnc_clr_p_bit(index, bit);
            break;
        case 'b':
        case 'B':
            _gnc_clr_b_bit(index, bit);
            break;
    }
}

bool get_para_bit(int No) {
    char reg;
    int index, bit;
    reg = PlcPara[No].type;
    index = PlcPara[No].idx;
    bit = PlcPara[No].bit;
    return GetRegBit(reg, index, bit);
}

void set_para_bit(int No) {
    char reg;
    int index, bit;
    reg = PlcPara[No].type;
    index = PlcPara[No].idx;
    bit = PlcPara[No].bit;
    SetRegBit(reg, index, bit);
}

void clr_para_bit(int No) {
    char reg;
    int index, bit;
    reg = PlcPara[No].type;
    index = PlcPara[No].idx;
    bit = PlcPara[No].bit;
    ClrRegBit(reg, index, bit);
}

long get_para_reg(int No, int n) {
    return GetReg(PlcPara[No].type, PlcPara[No].idx + n);
}

void set_para_reg(int No, long value) {
    SetReg(PlcPara[No].type, PlcPara[No].idx, value);
}

//************获取当前周期LDP等指令要用，缓存数据******************
static bool GetPNState(int ParaNo, bool flag) {
    int i, n;
    if (m_PNBuf == NULL) return false;
    n = (*m_PNBuf).size();
    for (i = 0; i < n; i++) {
        if (PlcPara[ParaNo].type == (*m_PNBuf)[i].reg_type && PlcPara[ParaNo].idx == (*m_PNBuf)[i].index && PlcPara[ParaNo].bit == (*m_PNBuf)[i].bit) {
            if (flag) {
                if ((*m_PNBuf)[i].last == 0 && (*m_PNBuf)[i].now == 1)
                    return true;
                else
                    return false;
            } else {
                if ((*m_PNBuf)[i].last == 1 && (*m_PNBuf)[i].now == 0)
                    return true;
                else
                    return false;
            }
        }
    }
    return false;
}

static void LD_run(int NO) {
    ST <<= 1;
    if (get_para_bit(NO))
        ST |= 0x1;
    else
        ST &= ~0x1;
}

static void LDI_run(int NO) {
    ST <<= 1;
    if (!get_para_bit(NO))
        ST |= 0x1;
    else
        ST &= ~0x1;
}

static void LDP_run(int NO) {
    ST <<= 1;
    if (GetPNState(NO, true))
        ST |= 0x1;
    else
        ST &= ~0x1;
}

static void LDF_run(int NO) {
    ST <<= 1;
    if (GetPNState(NO, false))
        ST |= 0x1;
    else
        ST &= ~0x1;
}

static void OUT_run(int NO) {
    if ((ST & 0x1) == 0x1) {
        set_para_bit(NO);
    } else {
        clr_para_bit(NO);
    }
}

static void OOUT_run(int NO) {
    if ((ST & 0x1) != 0x1)
        set_para_bit(NO);
    else
        clr_para_bit(NO);
}

static void SET_run(int NO) {
    if ((ST & 0x1) == 0x1)
        set_para_bit(NO);
}

static void RST_run(int NO) {
    if ((ST & 0x1) == 0x1) clr_para_bit(NO);
}

static void AND_run(int NO) {
    if ((ST & 0x1) != 0x1) return;
    if (!get_para_bit(NO)) ST &= ~0x1;
}

static void ANI_run(int NO) {
    if ((ST & 0x1) != 0x1) return;
    if (get_para_bit(NO)) ST &= ~0x1;
}

static void OR_run(int NO) {
    if ((ST & 0x1) == 0x1) return;
    if (get_para_bit(NO)) ST |= 0x1;
}

static void ORI_run(int NO) {
    if ((ST & 0x1) == 0x1) return;
    if (!get_para_bit(NO)) ST |= 0x1;
}

static void ANDP_run(int NO) {
    if ((ST & 0x1) != 0x1) return;
    if (!GetPNState(NO, true)) ST &= ~0x1;
}

static void ANDF_run(int NO) {
    if ((ST & 0x1) != 0x1) return;
    if (!GetPNState(NO, false)) ST &= ~0x1;
}

static void ORP_run(int NO) {
    if ((ST & 0x1) == 0x1) return;
    if (GetPNState(NO, true)) ST |= 0x1;
}

static void ORF_run(int NO) {
    if ((ST & 0x1) == 0x1) return;
    if (GetPNState(NO, false)) ST |= 0x1;
}

static void ANB_run(int NO) {
    if ((ST & 0x02) == 0x02) {
        if ((ST & 0x1) != 0x1) {
            ST &= ~0x02;
        }
    }
    ST >>= 1;
}

static void ORB_run(int NO) {
    if ((ST & 0x02) != 0x02) {
        if ((ST & 0x1) == 0x1) {
            ST |= 0x02;
        }
    }
    ST >>= 1;
}

static void MPS_run(int NO) {
    MP <<= 1;
    if ((ST & 0x01) == 0x01)
        MP |= 0X01;
    else
        MP &= ~0x01;
}

static void MRD_run(int NO) {
    if ((MP & 0x01) == 0x01)
        ST |= 0x01;
    else
        ST &= ~0x01;
}

static void MPP_run(int NO) {
    if ((MP & 0x01) == 0x01)
        ST |= 0x01;
    else
        ST &= ~0x01;
    MP >>= 1;
}

static void SP_run(int NO)
{

}

static void SPE_run(int NO)
{

}

static void IEND_run(int NO)
{
    return;
}

static void END1_run(int NO)
{
    return;
}

static void END2_run(int NO)
{
    return;
}

static void MOV_run(int NO)
{
    long a,b;
    if((ST&0x1)==0x1)
    {
        a=get_para_reg(NO);
        b=a;
        set_para_reg(NO+1,b);
    }
}

static void ADD_run(int NO)
{
    long a,b,c;
    if((ST&0x1) ==0x1)
    {
        a=get_para_reg(NO+1);
        b=get_para_reg(NO+2);
        c=a+b;
        set_para_reg(NO+3,c);
    }
}

static void SUB_run(int NO)
{
    long a,b,c;
    if((ST&0x1) ==0x1)
    {
        a=get_para_reg(NO+1);
        b=get_para_reg(NO+2);
        c=a-b;
        set_para_reg(NO+3,c);
        if(c>0)
            ST&=~0x01;
    }
}

static void MUL_run(int NO)
{
    long a,b,c;
    if((ST&0x1) ==0x1)
    {
        a=get_para_reg(NO+1);
        b=get_para_reg(NO+2);
        c=a*b;
        set_para_reg(NO+3,c);
    }
}

static void DIV_run(int NO)
{
    long a,b,c;
    if((ST&0x1) ==0x1)
    {
        a=get_para_reg(NO+1);
        b=get_para_reg(NO+2);
        if(b==0)
            return;
        else
            ST&=~0x01;
        c=a/b;
        set_para_reg(NO+3,c);
    }
}

static void MOD_run(int NO)
{
    long a,b,c;
    if((ST&0x1) ==0x1)
    {
        a=get_para_reg(NO+1);
        b=get_para_reg(NO+2);
        if(b==0)
            return;
        else
            ST&=~0x01;
        c=a%b;
        set_para_reg(NO+3,c);
    }
}

static void INC_run(int NO)
{
    long a;
    if((ST&0x1)==0x1)
    {
        a=get_para_reg(NO);
        a++;
        set_para_reg(NO,a);
    }
}

static void DEC_run(int NO)
{
    long a;
    if((ST&0x1)==0x1)
    {
        a=get_para_reg(NO);
        if(a>0)
        {
            a--;
            set_para_reg(NO,a);
            ST&=~0x01;
        }
    }
}

static void F_signal() {
    for (int i = 0; i < 128; ++i) {
        uchar value;
        getRegValue8(REG_F, i, &value);
        FF[i] = value;
    }
}

static void G_signal() {
    for (int i = 0; i < 128; ++i)
    {
        uint8* value = &GG[i];
        setRegValue8(REG_G, i, *value);
    }
}

static void I_map() {
    CurrentRegData.regX[4]  = plc_in.pd_data.keyboardIn1.all;      // MCP***+5/****/door
    CurrentRegData.regX[5]  = plc_in.pd_data.keyboardIn1.all >> 8; //-x
    CurrentRegData.regX[6]  = plc_in.pd_data.keyboardIn2.all;      // F5
    CurrentRegData.regX[7]  = plc_in.pd_data.keyboardIn2.all >> 8; //程序再启动
    CurrentRegData.regX[8]  = plc_in.pd_data.keyboardIn3.all;      //刀库反转
    CurrentRegData.regX[9]  = plc_in.pd_data.keyboardIn3.all >> 8; //排屑机正转
    CurrentRegData.regX[10] = plc_in.pd_data.keyboardIn4.all;
    CurrentRegData.regX[11] = plc_in.pd_data.keyboardIn4.all >> 8;
    CurrentRegData.regX[12] = plc_in.pd_data.keyboardIn5.all;
    CurrentRegData.regX[13] = plc_in.pd_data.keyboardIn5.all >> 8;
    CurrentRegData.regX[14] = plc_in.pd_data.keyboardIn6.all;
    CurrentRegData.regX[15] = plc_in.pd_data.keyboardIn6.all >> 8;
    CurrentRegData.regX[16] = plc_in.pd_data.keyboardIn7.all;
    CurrentRegData.regX[17] = plc_in.pd_data.keyboardIn7.all >> 8;
    //    CurrentRegData.regX[18]=plc_in.pd_data.keyboardIn8.all;
    //    CurrentRegData.regX[19]=plc_in.pd_data.keyboardIn8.all>>8;

    //    CurrentRegData.regX[20]=plc_in.pd_data.keyboardInb.all;
    //    CurrentRegData.regX[21]=plc_in.pd_data.keyboardInb.all>>8;

    //    CurrentRegData.regX[22]=plc_in.pd_data.keyboardIn9.all;
    //    CurrentRegData.regX[23]=plc_in.pd_data.keyboardIn9.all>>8;
    for (int j = 0; j < 8; ++j) {
        XX[j]      = abs(CurrentRegData.regX[j]);
        XX[8 + j]  = abs(CurrentRegData.regX[8 + j]);
        XX[16 + j] = abs(CurrentRegData.regX[16 + j]);
        XX[24 + j] = abs(CurrentRegData.regX[24 + j]);
    }
    F_signal();
}

static void O_map() {
    for (int j = 0; j < 8; ++j)
    {
        CurrentRegData.regY[j]      = abs(YY[j]);
        CurrentRegData.regY[j + 8]  = abs(YY[j + 8]);
        CurrentRegData.regY[j + 16] = abs(YY[j + 16]);
        CurrentRegData.regY[j + 24] = abs(YY[j + 24]);
    }
    plc_out.IO_Out1Reg.all = abs(CurrentRegData.regY[0]) | (abs(CurrentRegData.regY[1]) << 8);
    plc_out.panel_show.keyboardOut1.all = abs(CurrentRegData.regY[4]) | (abs(CurrentRegData.regY[5])  << 8);
    plc_out.panel_show.keyboardOut2.all = abs(CurrentRegData.regY[6]) | (abs(CurrentRegData.regY[7])  << 8);
    plc_out.panel_show.keyboardOut3.all = abs(CurrentRegData.regY[8]) | (abs(CurrentRegData.regY[9])  << 8);
    plc_out.panel_show.keyboardOut4.all = abs(CurrentRegData.regY[10])| (abs(CurrentRegData.regY[11]) << 8);
    plc_out.panel_show.keyboardOut5.all = abs(CurrentRegData.regY[12])| (abs(CurrentRegData.regY[13]) << 8); // LED_MAG
    plc_out.panel_show.keyboardOut6.all = abs(CurrentRegData.regY[14])| (abs(CurrentRegData.regY[15]) << 8); // LED_SPD
    plc_out.panel_show.keyboardOut7.all = abs(CurrentRegData.regY[16])| (abs(CurrentRegData.regY[17]) << 8);
    plc_out.panel_show.keyboardOut8.all = abs(CurrentRegData.regY[18])| (abs(CurrentRegData.regY[19]) << 8);
    G_signal();
}

static void row_run(int n) {
    if (array[PLCRun[n].opcode] != nullptr) {
        (*array[PLCRun[n].opcode])(PLCRun[n].regIdx);
    }
}

static void DealEndPNBufJumpCall() {
    int i, j;
    typePNEdge p;
    // int jump_num, call_num;
    // struct {
    //     long No;
    //     long Jump;
    //     long Label;
    // } JumpBuf[50]; //*****50个跳转指令********
    // struct {
    //     long parm[4];
    //     long Call;
    //     long sp;
    // } CallBuf[50];                       //******50个子程序调用指令****
    init_end = plc1_end = plc2_end = -1; //********默认值********
    // jump_num = call_num = 0;             //********默认值************
    plcRunNum = getLadDataPtr()->info.instrCount;
    for (i = 0; i < plcRunNum; i++) {
        switch (PLCRun[i].opcode) {
            case OP_IEND:
                init_end = i;
                break;
            case OP_1END:
                plc1_end = i;
                break;
            case OP_2END:
                plc2_end = i;
                break;
            case OP_LDP:
            case OP_LDF:
            case OP_ORP:
            case OP_ORF:
            case OP_ANDF:
            case OP_ANDP:
                memset(&p, 0, sizeof(typePNEdge));
                p.reg_type = PlcPara[PLCRun[i].opcode].type;
                p.index = PlcPara[PLCRun[i].opcode].idx;
                p.bit = PlcPara[PLCRun[i].opcode].bit;
                for (j = 0; j < m_PN1Buf.size(); j++) {
                    if (p.reg_type == m_PN1Buf[i].reg_type && p.index == m_PN1Buf[i].index && p.bit == m_PN1Buf[i].bit) break;
                }
                if (j == m_PN1Buf.size()) {
                    m_PN1Buf.append(p);
                    m_PN2Buf.append(p);
                }
                //        case OP_JMP:
                //            for (j=0;j<jmp_num;j++)
                //            {
                //               if(JumpBuf[j].No==PlcPara[PLCRun[i].opcode].idx)
                //               {
                //                   PlcPara[PLCRun[i].opcode].idx=JumpBuf[j].Label;
                //                   break;
                //               }
                //            }
                //            if(j>=jump_num)
                //            {
                //                JumpBuf[jump_num].No=PlcPara[PLCRun[i].opcode].idx;
                //                JumpBuf[jump_num].Jump=i;
                //            }
                //            break;
                //        case OP_LBL:
                //            for (j=0;j<jump_num;j++)
                //            {
                //                if(JumpBuf[j].No==PlcPara[PLCRun[i].opcode].idx)
                //                {
                //                    PlcPara[PLCRun[JumpBuf[j].Jump].opcode].idx=i;
                //                    break;
                //                }
                //                if(j>=jump_num)
                //                {
                //                    JumpBuf[jump_num].No=PlcPara[PLCRun[i].opcode].idx;
                //                    JumpBuf[jump_num].Label=i;
                //                }
                //            }
                //                break;
                //        case OP_CALL:
                //            break;
            case OP_SP:
                break;
            default:
                break;
        }
    }
}

static void copy_IL() {
    memset(PLCRun, 0, sizeof(PLCRun));
    memset(PlcPara, 0, sizeof(PlcPara));
    memcpy(PLCRun, Ladderdata->instrs, sizeof(LadInstr) * 10000);
    memcpy(PlcPara, Ladderdata->regs, sizeof(LadReg) * 24000);
}

//************指令初始化*************************
static void plc_run_init() {
    copy_IL();
    DealEndPNBufJumpCall();
    // qDebug()<<"************梯图信息,指令数据测试90*************";
    array[OP_LD] = LD_run;
    array[OP_LDI] = LDI_run;
    array[OP_OUT] = OUT_run;
    array[OP_OOUT] = OOUT_run;
    array[OP_SET] = SET_run;
    array[OP_RST] = RST_run;
    array[OP_AND] = AND_run;
    array[OP_ANI] = ANI_run;
    array[OP_OR] = OR_run;
    array[OP_ORI] = ORI_run;
    array[OP_LDP] = LDP_run;
    array[OP_LDF] = LDF_run;
    array[OP_ANDP] = ANDP_run;
    array[OP_ANDF] = ANDF_run;
    array[OP_ORP] = ORP_run;
    array[OP_ORF] = ORF_run;
    array[OP_ORB] = ORB_run;
    array[OP_ANB] = ANB_run;
    array[OP_MPS] = MPS_run;
    array[OP_MRD] = MRD_run;
    array[OP_MPP] = MPP_run;
    array[OP_IEND] = IEND_run;
    array[OP_1END] = END1_run;
    array[OP_2END] = END2_run;
    array[OP_SP]   = SP_run;
    array[OP_SPE]  = SPE_run;
    array[OP_ADD] = ADD_run;
    array[OP_SUB] = SUB_run;
    array[OP_MUL] = MUL_run;
    array[OP_DIV] = DIV_run;
    array[OP_INC] = INC_run;
    array[OP_DEC] = DEC_run;
//    array[OP_WAND] = WAND_run;
//    array[OP_WOR] = WOR_run;
//    array[OP_WXOR] = WXOR_run;
//    array[OP_SHL] = SHL_run;
//    array[OP_SHR] = SHR_run;
//    array[OP_ROL] = ROL_run;
//    array[OP_ROR] = ROR_run;
    array[OP_MOV] = MOV_run;
//    array[OP_TMR] = TMR_run;
//    array[OP_CTR] = CTR_run;
//    array[OP_JMP] = JMP_run;
//    array[OP_LBL] = LBL_run;
//    array[OP_CALL] = CALL_run;
//    array[OP_TOTAL] = TOTAL_run;
}

//************记录上周期数据及更新当前周期数据（LDP、LDF等）******************
static void RefplcPNBuf() {
    int i, n;
    n = (*m_PNBuf).size();
    for (i = 0; i < n; i++) {
        (*m_PNBuf)[i].last = (*m_PNBuf)[i].now;
        if (GetRegBit((*m_PNBuf)[i].reg_type, (*m_PNBuf)[i].index, (*m_PNBuf)[i].bit))
            (*m_PNBuf)[i].now = 1;
        else
            (*m_PNBuf)[i].now = 0;
    }
}

static void RunInit() {
    m_PNBuf = nullptr;
    plc_run_init();
    if (init_end > 0) {
        run_row = 0;
        while (run_row != init_end) {
            row_run(run_row);
            run_row++;
        }
    }
}

static void RunPlc1() {
    m_PNBuf = &m_PN1Buf;
    run_row = init_end;
    RefplcPNBuf();
    if (plc1_end > 0 && run_row < plc1_end) {
        if (run_row < 0) run_row = 0;
        while (run_row != plc1_end) {
            row_run(run_row);
            run_row++;
        }
    }
}

static void RunPlc2() {
    m_PNBuf = &m_PN2Buf;
    run_row = plc1_end;
    RefplcPNBuf();
    if (plc2_end > 0 && run_row < plc2_end) {
        if (run_row < 0) run_row = 0;
        while (run_row != plc2_end) {
            row_run(run_row);
            run_row++;
        }
    }
}

void CycleScanning() {
    if (!init_system) {
        I_map();
        RunInit();
        O_map();
        init_system = 1;
    }
    if (init_system) {
        I_map();
        RunPlc1();
        RunPlc2();
        O_map();
    }
}

// static void test1()
// {



// }

// static void test2()
// {



// }

// static void test3()
// {



// }

// static void test4()
// {



// }


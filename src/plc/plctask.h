#pragma once
#include "laddata.h"
#include <stack>
#include <vector>

struct PlcContext {
    int pc;                     // 当前指针
    int begin;                  // 起始位置
    int end;                    // 终点位置
    uint32_t _logicStack;       // 逻辑运算栈，最多支持32层嵌套
    std::stack<int> _callStack; // 调用栈，存储返回地址
};

struct EdgeState {
    bool lastState;
    bool currState;
    int instrIdx;
    EdgeState(bool last, bool curr, int idx) : lastState(last), currState(curr), instrIdx(idx) {}
};

class PlcTask {
public:
    PlcTask(const PlcTask&) = delete;
    PlcTask(PlcTask&&) = delete;
    PlcTask& operator=(const PlcTask&) = delete;
    PlcTask& operator=(PlcTask&&) = delete;
    static PlcTask& instance();
    
    explicit PlcTask();
    void execute();

private:
    LadData& _ladData;
    RegData& _regData;
    bool _isRunning;
    PlcContext _initCtx;
    PlcContext _plc1Ctx;
    PlcContext _plc2Ctx;
    PlcContext* _currCtx;
    std::vector<EdgeState> _edgeStates; // 上升沿下降沿状态
    std::vector<int> _edgeIdxs; // 上升沿下降沿索引
    typedef void (PlcTask::*OpHandler)(const LadInstr&);
    OpHandler _dispatchTable[OP_TOTAL];
    int timers[REG_T_NUM] = {0};
    int _subprgBegin[LAD_SUBPRG_MAX_NUM - 3] = {-1};
    int _labelPositions[LAD_MAX_LABEL_NUM] = {-1};
    int _cycleCount = 0;

    void initDispatchTable();
    void resetContext();
    void executeInit();
    void executePlc1();
    void executePlc2();
    void updatePlc1LastEdge();
    void updatePlc1CurrEdge();
    void updatePlc2LastEdge();
    void updatePlc2CurrEdge();
    void updateTimer();

    inline void handleDefault(const LadInstr& instr);
    inline void handleLDT(const LadInstr& instr);
    inline void handleLD(const LadInstr& instr);
    inline void handleLDI(const LadInstr& instr);
    inline void handleAND(const LadInstr& instr);
    inline void handleANI(const LadInstr& instr);
    inline void handleOR(const LadInstr& instr);
    inline void handleORI(const LadInstr& instr);
    inline void handleLDP(const LadInstr& instr);
    inline void handleLDF(const LadInstr& instr);
    inline void handleANDP(const LadInstr& instr);
    inline void handleANDF(const LadInstr& instr);
    inline void handleORP(const LadInstr& instr);
    inline void handleORF(const LadInstr& instr);
    inline void handleOUT(const LadInstr& instr);
    inline void handleOOUT(const LadInstr& instr);
    inline void handleSET(const LadInstr& instr);
    inline void handleRST(const LadInstr& instr);
    inline void handleANB(const LadInstr& instr);
    inline void handleORB(const LadInstr& instr);
    inline void handleMPS(const LadInstr& instr);
    inline void handleMRD(const LadInstr& instr);
    inline void handleMPP(const LadInstr& instr);
    inline void handleSP(const LadInstr& instr);
    inline void handleSPE(const LadInstr& instr);
    inline void handleCALL(const LadInstr& instr);
    inline void handleRETN(const LadInstr& instr);
    inline void handleJMP(const LadInstr& instr);
    inline void handleLBL(const LadInstr& instr);
    inline void handleADD(const LadInstr& instr);
    inline void handleSUB(const LadInstr& instr);
    inline void handleMUL(const LadInstr& instr);
    inline void handleDIV(const LadInstr& instr);
    inline void handleMOD(const LadInstr& instr);
    inline void handleINC(const LadInstr& instr);
    inline void handleDEC(const LadInstr& instr);
    inline void handleABS(const LadInstr& instr);
    inline void handleNEG(const LadInstr& instr);
    inline void handleEQ(const LadInstr& instr);
    inline void handleNEQ(const LadInstr& instr);
    inline void handleGT(const LadInstr& instr);
    inline void handleLT(const LadInstr& instr);
    inline void handleGTE(const LadInstr& instr);
    inline void handleLTE(const LadInstr& instr);
    inline void handleRNG(const LadInstr& instr);
    inline void handleXOR(const LadInstr& instr);
    inline void handleWAND(const LadInstr& instr);
    inline void handleWOR(const LadInstr& instr);
    inline void handleNOT(const LadInstr& instr);
    inline void handleSHL(const LadInstr& instr);
    inline void handleSHR(const LadInstr& instr);
    inline void handleROL(const LadInstr& instr);
    inline void handleROR(const LadInstr& instr);
    inline void handleTMRB(const LadInstr& instr);
    inline void handleSTMR(const LadInstr& instr);
    inline void handleCTR(const LadInstr& instr);
    inline void handleCTRC(const LadInstr& instr);
    inline void handleCTUD(const LadInstr& instr);
    inline void handleMOV(const LadInstr& instr);
    inline void handleROT(const LadInstr& instr);

    inline void setBit(LadReg& reg, bool value);
    inline bool getBit(LadReg& reg);
    inline void setBit(LadReg& reg);
    inline void clearBit(LadReg& reg);
    inline int getValue(LadReg& reg);
    inline void setValue(LadReg& reg, int value);
};
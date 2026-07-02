#include <algorithm>
#include <climits>
#include "plctask.h"
#include "ladapi.h"
#include "laddata.h"
#include "regapi.h"
#include "regdata.h"

PlcTask::PlcTask() 
    : _ladData(*getLadDataPtr())
    , _regData(*getRegDataPtr()){
    initDispatchTable();
}

PlcTask& PlcTask::instance() {
    static PlcTask _instance;
    return _instance;
}

void PlcTask::execute() {
    // 如果运行状态变停止
    if (_isRunning && !_ladData.info.isRunning) {
        _isRunning = false;
    } 
    // 如果停止状态变运行
    if (!_isRunning && _ladData.info.isRunning) {
        // resetRegData();
        resetContext();
        executeInit();
        _isRunning = true;
    }
    if (_isRunning) {
        executePlc1();
        executePlc2();
    }
}

void PlcTask::initDispatchTable() {
    for (int i = 0; i < OP_TOTAL; i++) {
        _dispatchTable[i] = &PlcTask::handleDefault;
    }
    _dispatchTable[OP_LDT] = &PlcTask::handleLDT;
    _dispatchTable[OP_LD] = &PlcTask::handleLD;
    _dispatchTable[OP_LDI] = &PlcTask::handleLDI;
    _dispatchTable[OP_AND] = &PlcTask::handleAND;
    _dispatchTable[OP_ANI] = &PlcTask::handleANI;
    _dispatchTable[OP_OR] = &PlcTask::handleOR;
    _dispatchTable[OP_ORI] = &PlcTask::handleORI;
    _dispatchTable[OP_LDP] = &PlcTask::handleLDP;
    _dispatchTable[OP_LDF] = &PlcTask::handleLDF;
    _dispatchTable[OP_ANDP] = &PlcTask::handleANDP;
    _dispatchTable[OP_ANDF] = &PlcTask::handleANDF;
    _dispatchTable[OP_ORP] = &PlcTask::handleORP;
    _dispatchTable[OP_ORF] = &PlcTask::handleORF;
    _dispatchTable[OP_PLS] = &PlcTask::handlePLS;
    _dispatchTable[OP_PLF] = &PlcTask::handlePLF;
    _dispatchTable[OP_ANB] = &PlcTask::handleANB;
    _dispatchTable[OP_ORB] = &PlcTask::handleORB;
    _dispatchTable[OP_MPS] = &PlcTask::handleMPS;
    _dispatchTable[OP_MRD] = &PlcTask::handleMRD;
    _dispatchTable[OP_MPP] = &PlcTask::handleMPP;
    _dispatchTable[OP_OUT] = &PlcTask::handleOUT;
    _dispatchTable[OP_OOUT] = &PlcTask::handleOOUT;
    _dispatchTable[OP_SET] = &PlcTask::handleSET;
    _dispatchTable[OP_RST] = &PlcTask::handleRST;
    _dispatchTable[OP_SP] = &PlcTask::handleSP;
    _dispatchTable[OP_SPE] = &PlcTask::handleSPE;
    _dispatchTable[OP_CALL] = &PlcTask::handleCALL;
    _dispatchTable[OP_RETN] = &PlcTask::handleRETN;
    _dispatchTable[OP_JMP] = &PlcTask::handleJMP;
    _dispatchTable[OP_LBL] = &PlcTask::handleLBL;
    _dispatchTable[OP_LOOP] = &PlcTask::handleLOOP;
    _dispatchTable[OP_NEXT] = &PlcTask::handleNEXT;
    // 算术
    _dispatchTable[OP_ADD] = &PlcTask::handleADD;
    _dispatchTable[OP_SUB] = &PlcTask::handleSUB;
    _dispatchTable[OP_MUL] = &PlcTask::handleMUL;
    _dispatchTable[OP_DIV] = &PlcTask::handleDIV;
    _dispatchTable[OP_MOD] = &PlcTask::handleMOD;
    _dispatchTable[OP_INC] = &PlcTask::handleINC;
    _dispatchTable[OP_DEC] = &PlcTask::handleDEC;
    _dispatchTable[OP_ABS] = &PlcTask::handleABS;
    _dispatchTable[OP_NEG] = &PlcTask::handleNEG;
    // 比较
    _dispatchTable[OP_EQ]  = &PlcTask::handleEQ;
    _dispatchTable[OP_NEQ] = &PlcTask::handleNEQ;
    _dispatchTable[OP_GT]  = &PlcTask::handleGT;
    _dispatchTable[OP_LT]  = &PlcTask::handleLT;
    _dispatchTable[OP_GTE] = &PlcTask::handleGTE;
    _dispatchTable[OP_LTE] = &PlcTask::handleLTE;
    _dispatchTable[OP_RNG] = &PlcTask::handleRNG;
    // 位操作
    _dispatchTable[OP_XOR]  = &PlcTask::handleXOR;
    _dispatchTable[OP_WAND] = &PlcTask::handleWAND;
    _dispatchTable[OP_WOR]  = &PlcTask::handleWOR;
    _dispatchTable[OP_NOT]  = &PlcTask::handleNOT;
    _dispatchTable[OP_SHL]  = &PlcTask::handleSHL;
    _dispatchTable[OP_SHR]  = &PlcTask::handleSHR;
    _dispatchTable[OP_ROL]  = &PlcTask::handleROL;
    _dispatchTable[OP_ROR]  = &PlcTask::handleROR;
    // 定时器
    _dispatchTable[OP_TMRB] = &PlcTask::handleTMRB;
    _dispatchTable[OP_STMR] = &PlcTask::handleSTMR;
    // 计数器
    _dispatchTable[OP_CTR] = &PlcTask::handleCTR;
    _dispatchTable[OP_CTRC] = &PlcTask::handleCTRC;
    _dispatchTable[OP_CTUD] = &PlcTask::handleCTUD;
    // 移位操作
    _dispatchTable[OP_MOV]  = &PlcTask::handleMOV;
    _dispatchTable[OP_XMOV] = &PlcTask::handleXMOV;
    _dispatchTable[OP_BMOV] = &PlcTask::handleBMOV;
    _dispatchTable[OP_FMOV] = &PlcTask::handleFMOV;
    _dispatchTable[OP_SER] = &PlcTask::handleSER;
    _dispatchTable[OP_ASSEM] = &PlcTask::handleASSEM;
    _dispatchTable[OP_DISAS] = &PlcTask::handleDISAS;
    _dispatchTable[OP_ALT] = &PlcTask::handleALT;
    // 旋转操作
    _dispatchTable[OP_ROT] = &PlcTask::handleROT;
}

void PlcTask::resetContext() {
    _edgeStates.clear();
    _edgeIdxs.assign(_ladData.info.instrCount, -1);
    _altStates.assign(_ladData.info.instrCount, 0);
    _loopNexts.assign(_ladData.info.instrCount, -1);
    _nextLoops.assign(_ladData.info.instrCount, -1);
    _loopCounts.assign(_ladData.info.instrCount, 0);
    std::stack<int> loopStack;
    for (int i = 0; i < _ladData.info.instrCount; i++) {
        switch (_ladData.instrs[i].opcode) {
            case OP_IEND:
                _initCtx.end = i;
                break;
            case OP_1END:
                _plc1Ctx.end = i;
                break;
            case OP_2END:
                _plc2Ctx.end = i;
                break;
            case OP_LDP:
            case OP_LDF:
            case OP_ANDP:
            case OP_ANDF:
            case OP_ORP:
            case OP_ORF: {
                _edgeIdxs[i] = _edgeStates.size(); 
                bool bitVal = getBit(_ladData.regs[_ladData.instrs[i].regIdx]);
                _edgeStates.emplace_back(bitVal, bitVal, i, false);
                break;
            }
            case OP_PLS:
            case OP_PLF:
            case OP_CTR:
            case OP_CTRC:
            case OP_CTUD:
            case OP_ALT: {
                _edgeIdxs[i] = _edgeStates.size();
                _edgeStates.emplace_back(false, false, i, true);
                break;
            }
            case OP_SP: 
                // 子程序起始位置记录在 _subprgBegin 数组中
                _subprgBegin[_ladData.regs[_ladData.instrs[i].regIdx].idx] = i; 
                break;
            case OP_LBL: 
                // 标签位置记录在 _labelPositions 数组中
                _labelPositions[_ladData.regs[_ladData.instrs[i].regIdx].idx] = i;
                break;
            case OP_LOOP:
                loopStack.push(i);
                break;
            case OP_NEXT:
                Q_ASSERT(!loopStack.empty());
                if (!loopStack.empty()) {
                    int loopIdx = loopStack.top();
                    loopStack.pop();
                    _loopNexts[loopIdx] = i;
                    _nextLoops[i] = loopIdx;
                }
                break;
            default:
                break;
        }
    }
    Q_ASSERT(loopStack.empty());

    _initCtx.begin = 0;
    _plc1Ctx.begin = _initCtx.end + 1;
    _plc2Ctx.begin = _plc1Ctx.end + 1;
    _initCtx.pc = _initCtx.begin;
    _plc1Ctx.pc = _plc1Ctx.begin;
    _plc2Ctx.pc = _plc2Ctx.begin;
}

void PlcTask::executeInit() {
    _currCtx = &_initCtx;
    while (_initCtx.pc <= _initCtx.end) {
        LadInstr& instr = _ladData.instrs[_initCtx.pc];
        (this->*_dispatchTable[instr.opcode])(instr);
        _initCtx.pc++;
    }
    _initCtx.pc = _initCtx.begin;
}

void PlcTask::executePlc1() {
    updateTimer();
    updatePlc1CurrEdge();
    _currCtx = &_plc1Ctx;
    while (_plc1Ctx.pc <= _plc1Ctx.end) {
        LadInstr& instr = _ladData.instrs[_plc1Ctx.pc];
        (this->*_dispatchTable[instr.opcode])(instr);
        _plc1Ctx.pc++;
    }
    _plc1Ctx.pc = _plc1Ctx.begin;
    updatePlc1LastEdge();
}
    
void PlcTask::executePlc2() {
    _currCtx = &_plc2Ctx;
    _cycleCount++;
    if (_plc2Ctx.pc == _plc2Ctx.begin) {
        updatePlc2CurrEdge();
    }
    for (int i = 0; i < 500 && _plc2Ctx.pc != _plc2Ctx.end; i++) {
        LadInstr& instr = _ladData.instrs[_plc2Ctx.pc];
        (this->*_dispatchTable[instr.opcode])(instr);
        _plc2Ctx.pc++;
    }
    if (_plc2Ctx.pc == _plc2Ctx.end) {
        updatePlc2LastEdge();
        _ladData.info.plc2CurCycle = _cycleCount;
        _cycleCount = 0;
        _plc2Ctx.pc = _plc2Ctx.begin;
    }
}

void PlcTask::updatePlc1LastEdge() {
    for (size_t i = 0; i < _edgeStates.size(); i++) {
        if (_edgeStates[i].instrIdx <= _plc1Ctx.end) {
            _edgeStates[i].lastState = _edgeStates[i].currState;
        }
    }
}

void PlcTask::updatePlc1CurrEdge() {
    for (size_t i = 0; i < _edgeStates.size(); i++) {
        if (_edgeStates[i].instrIdx <= _plc1Ctx.end) {
            if (_edgeStates[i].isFlow) {
                _edgeStates[i].currState = false;
            } else {
                int& regIdx = _ladData.instrs[_edgeStates[i].instrIdx].regIdx;
                _edgeStates[i].currState = getBit(_ladData.regs[regIdx]);
            }
        }
    }
}

void PlcTask::updatePlc2LastEdge() {
    for (size_t i = 0; i < _edgeStates.size(); i++) {
        if (_edgeStates[i].instrIdx >= _plc2Ctx.begin) {
            _edgeStates[i].lastState = _edgeStates[i].currState;
        }
    }
}

void PlcTask::updatePlc2CurrEdge() {
    for (size_t i = 0; i < _edgeStates.size(); i++) {
        if (_edgeStates[i].instrIdx >= _plc2Ctx.begin) {
            if (_edgeStates[i].isFlow) {
                _edgeStates[i].currState = false;
            } else {
                int& regIdx = _ladData.instrs[_edgeStates[i].instrIdx].regIdx;
                _edgeStates[i].currState = getBit(_ladData.regs[regIdx]);
            }
        }
    }
}

void PlcTask::updateTimer() {
    for(int i = 0; i < REG_T_NUM; i++) {
        if (_ladData.timerTypes[i] != T_TYPE_NONE) {
            timers[i]++;
        }
    }
}

void PlcTask::handleDefault(const LadInstr& instr) {
    Q_UNUSED(instr)
}

void PlcTask::handleLDT(const LadInstr& instr) {
    Q_UNUSED(instr)
    pushLogic(true);
}

void PlcTask::handleLD(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    pushLogic(bitVal);
}

void PlcTask::handleLDI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    pushLogic(!bitVal);
}

void PlcTask::handleAND(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    andLogicTop(bitVal);
}

void PlcTask::handleANI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    andLogicTop(!bitVal);
}

void PlcTask::handleOR(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    orLogicTop(bitVal);
}

void PlcTask::handleORI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    orLogicTop(!bitVal);
}

EdgeState& PlcTask::edgeState(int instrIdx) {
    Q_ASSERT(instrIdx >= 0 && instrIdx < static_cast<int>(_edgeIdxs.size()));
    int edgeIdx = _edgeIdxs[instrIdx];
    Q_ASSERT(edgeIdx >= 0 && edgeIdx < static_cast<int>(_edgeStates.size()));
    return _edgeStates[edgeIdx];
}

bool PlcTask::isRisingEdge(int instrIdx) {
    EdgeState& state = edgeState(instrIdx);
    return !state.lastState && state.currState;
}

bool PlcTask::isFallingEdge(int instrIdx) {
    EdgeState& state = edgeState(instrIdx);
    return state.lastState && !state.currState;
}

bool PlcTask::updateFlowRisingEdge(int instrIdx, bool currState) {
    EdgeState& state = edgeState(instrIdx);
    Q_ASSERT(state.isFlow);
    state.currState = currState;
    return isRisingEdge(instrIdx);
}

bool PlcTask::updateFlowFallingEdge(int instrIdx, bool currState) {
    EdgeState& state = edgeState(instrIdx);
    Q_ASSERT(state.isFlow);
    state.currState = currState;
    return isFallingEdge(instrIdx);
}

bool PlcTask::logicTop() const {
    return static_cast<bool>(_currCtx->_logicStack & 1);
}

bool PlcTask::logicAt(int offset) const {
    return static_cast<bool>((_currCtx->_logicStack >> offset) & 1);
}

void PlcTask::pushLogic(bool value) {
    _currCtx->_logicStack <<= 1;
    setLogicTop(value);
}

bool PlcTask::popLogic() {
    bool value = logicTop();
    _currCtx->_logicStack >>= 1;
    return value;
}

void PlcTask::setLogicTop(bool value) {
    if (value) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::replaceLogicTop(bool value) {
    setLogicTop(value);
}

void PlcTask::andLogicTop(bool value) {
    replaceLogicTop(logicTop() && value);
}

void PlcTask::orLogicTop(bool value) {
    replaceLogicTop(logicTop() || value);
}

PlcTask::CtrInputs PlcTask::popCtrInputs() {
    CtrInputs inputs;
    inputs.rst = popLogic();
    inputs.upd = popLogic();
    inputs.cn0 = popLogic();
    inputs.act = logicTop();
    return inputs;
}

PlcTask::CtrcInputs PlcTask::popCtrcInputs() {
    CtrcInputs inputs;
    inputs.rst = popLogic();
    inputs.act = logicTop();
    return inputs;
}

PlcTask::RotInputs PlcTask::popRotInputs() {
    RotInputs inputs;
    inputs.inc = popLogic();
    inputs.pos = popLogic();
    inputs.dir = popLogic();
    inputs.rn0 = popLogic();
    inputs.act = logicTop();
    return inputs;
}

int PlcTask::regCount(RegType type) {
    switch (type) {
        case REG_A:
            return REG_A_NUM;
        case REG_R:
            return REG_R_NUM;
        case REG_D:
            return REG_D_NUM;
        case REG_X:
            return REG_X_NUM;
        case REG_Y:
            return REG_Y_NUM;
        case REG_F:
            return REG_F_NUM;
        case REG_G:
            return REG_G_NUM;
        case REG_I:
            return REG_I_NUM;
        case REG_Q:
            return REG_Q_NUM;
        case REG_B:
            return REG_B_NUM;
        case REG_K:
            return REG_K_NUM;
        default:
            return 0;
    }
}

bool PlcTask::isRegRangeValid(const LadReg& reg, int offset, int count) {
    return reg.idx + offset + count <= regCount(reg.type);
}

int PlcTask::byteRegCount(RegType type) {
    switch (type) {
        case REG_A:
            return REG_A_NUM;
        case REG_R:
            return REG_R_NUM;
        case REG_X:
            return REG_X_NUM;
        case REG_Y:
            return REG_Y_NUM;
        case REG_F:
            return REG_F_NUM;
        case REG_G:
            return REG_G_NUM;
        case REG_I:
            return REG_I_NUM;
        case REG_Q:
            return REG_Q_NUM;
        case REG_K:
            return REG_K_NUM;
        default:
            return 0;
    }
}

int PlcTask::wordRegCount(RegType type) {
    switch (type) {
        case REG_D:
            return REG_D_NUM;
        case REG_B:
            return REG_B_NUM;
        default:
            return 0;
    }
}

unsigned char PlcTask::getByteValue(const LadReg& reg, int offset) {
    int idx = reg.idx + offset;
    if (offset < 0 || reg.idx < 0 || idx < 0 || idx >= byteRegCount(reg.type)) {
        return 0;
    }

    switch (reg.type) {
        case REG_A:
            return static_cast<unsigned char>(_regData.regA[idx]);
        case REG_R:
            return static_cast<unsigned char>(_regData.regR[idx]);
        case REG_X:
            return static_cast<unsigned char>(_regData.regX[idx]);
        case REG_Y:
            return static_cast<unsigned char>(_regData.regY[idx]);
        case REG_F:
            return static_cast<unsigned char>(_regData.regF[idx]);
        case REG_G:
            return static_cast<unsigned char>(_regData.regG[idx]);
        case REG_I:
            return static_cast<unsigned char>(_regData.regI[idx]);
        case REG_Q:
            return static_cast<unsigned char>(_regData.regQ[idx]);
        case REG_K:
            return static_cast<unsigned char>(_regData.regK[idx]);
        default:
            return 0;
    }
}

void PlcTask::setByteValue(const LadReg& reg, int offset, unsigned char value) {
    int idx = reg.idx + offset;
    if (offset < 0 || reg.idx < 0 || idx < 0 || idx >= byteRegCount(reg.type)) {
        return;
    }

    switch (reg.type) {
        case REG_A:
            _regData.regA[idx] = static_cast<char>(value);
            break;
        case REG_R:
            _regData.regR[idx] = static_cast<char>(value);
            break;
        case REG_X:
            _regData.regX[idx] = static_cast<char>(value);
            break;
        case REG_Y:
            _regData.regY[idx] = static_cast<char>(value);
            break;
        case REG_F:
            _regData.regF[idx] = static_cast<char>(value);
            break;
        case REG_G:
            _regData.regG[idx] = static_cast<char>(value);
            break;
        case REG_I:
            _regData.regI[idx] = static_cast<char>(value);
            break;
        case REG_Q:
            _regData.regQ[idx] = static_cast<char>(value);
            break;
        case REG_K:
            _regData.regK[idx] = static_cast<char>(value);
            break;
        default:
            break;
    }
}

uint32_t PlcTask::getWordValue(const LadReg& reg) {
    if (reg.idx < 0 || reg.idx >= wordRegCount(reg.type)) {
        return 0;
    }

    switch (reg.type) {
        case REG_D:
            return static_cast<uint32_t>(_regData.regD[reg.idx]);
        case REG_B:
            return static_cast<uint32_t>(_regData.regB[reg.idx]);
        default:
            return 0;
    }
}

void PlcTask::setWordValue(const LadReg& reg, uint32_t value) {
    if (reg.idx < 0 || reg.idx >= wordRegCount(reg.type)) {
        return;
    }

    switch (reg.type) {
        case REG_D:
            _regData.regD[reg.idx] = static_cast<int>(value);
            break;
        case REG_B:
            _regData.regB[reg.idx] = static_cast<int>(value);
            break;
        default:
            break;
    }
}

void PlcTask::handleLDP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isRisingEdge(_currCtx->pc);
    }
    pushLogic(bitVal);
}

void PlcTask::handleLDF(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isFallingEdge(_currCtx->pc);
    }
    pushLogic(bitVal);
}

void PlcTask::handleANDP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isRisingEdge(_currCtx->pc);
    }
    andLogicTop(bitVal);
}

void PlcTask::handleANDF(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isFallingEdge(_currCtx->pc);
    }
    andLogicTop(bitVal);
}

void PlcTask::handleORP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isRisingEdge(_currCtx->pc);
    }
    orLogicTop(bitVal);
}

void PlcTask::handleORF(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bitVal = isFallingEdge(_currCtx->pc);
    }
    orLogicTop(bitVal);
}

void PlcTask::handlePLS(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool currState = logicTop();
    replaceLogicTop(updateFlowRisingEdge(_currCtx->pc, currState));
}

void PlcTask::handlePLF(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool currState = logicTop();
    replaceLogicTop(updateFlowFallingEdge(_currCtx->pc, currState));
}

void PlcTask::handleOUT(const LadInstr& instr) {
    bool bitVal = logicTop();
    setBit(_ladData.regs[instr.regIdx], bitVal);
}

void PlcTask::handleOOUT(const LadInstr& instr) {
    bool bitVal = logicTop();
    setBit(_ladData.regs[instr.regIdx], !bitVal);
}

void PlcTask::handleSET(const LadInstr& instr) {
    bool bitVal = logicTop();
    if (bitVal) {
        setBit(_ladData.regs[instr.regIdx]);
    }
}

void PlcTask::handleRST(const LadInstr& instr) {
    bool bitVal = logicTop();
    if (bitVal) {
        clearBit(_ladData.regs[instr.regIdx]);
    }
}

void PlcTask::handleANB(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool bitVal = popLogic();
    andLogicTop(bitVal);
}

void PlcTask::handleORB(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool bitVal = popLogic();
    orLogicTop(bitVal);
}

void PlcTask::handleMPS(const LadInstr& instr) {
    Q_UNUSED(instr);
    pushLogic(logicTop());
}

void PlcTask::handleMRD(const LadInstr& instr) {
    Q_UNUSED(instr);
    replaceLogicTop(logicAt(1));
}

void PlcTask::handleMPP(const LadInstr& instr) {
    Q_UNUSED(instr);
    popLogic();
}

void PlcTask::handleSP(const LadInstr& instr) {
    Q_UNUSED(instr);
}

void PlcTask::handleSPE(const LadInstr& instr) {
    Q_UNUSED(instr);
    Q_ASSERT(!_currCtx->_callStack.empty());
    _currCtx->pc = _currCtx->_callStack.top();
    _currCtx->_callStack.pop();
}

void PlcTask::handleCALL(const LadInstr& instr) {
    if (logicTop()) {
        // 将当前 PC 入栈
        _currCtx->_callStack.push(_currCtx->pc);
        Q_ASSERT(_subprgBegin[_ladData.regs[instr.regIdx].idx] != -1);
        // 跳转到子程序开始
        _currCtx->pc = _subprgBegin[_ladData.regs[instr.regIdx].idx];
    }
}

void PlcTask::handleRETN(const LadInstr& instr) {
    Q_UNUSED(instr);
    Q_ASSERT(!_currCtx->_callStack.empty());
    if (logicTop()) {
        _currCtx->pc = _currCtx->_callStack.top();
        _currCtx->_callStack.pop();
    }
}

void PlcTask::handleLBL(const LadInstr& instr) {
    Q_UNUSED(instr);
}

void PlcTask::handleJMP(const LadInstr& instr) {
    if (logicTop()) {
        Q_ASSERT(_labelPositions[_ladData.regs[instr.regIdx].idx] != -1);
        // 跳转到标签位置
        _currCtx->pc = _labelPositions[_ladData.regs[instr.regIdx].idx];
    }
}

void PlcTask::handleLOOP(const LadInstr& instr) {
    int loopIdx = _currCtx->pc;
    int nextIdx = _loopNexts[loopIdx];
    Q_ASSERT(nextIdx != -1);

    if (!logicTop()) {
        _loopCounts[loopIdx] = 0;
        _currCtx->pc = nextIdx;
        return;
    }

    if (_loopCounts[loopIdx] == 0) {
        int count = getValue(_ladData.regs[instr.regIdx]);
        if (count <= 0) {
            _currCtx->pc = nextIdx;
            return;
        }
        _loopCounts[loopIdx] = count;
    }
}

void PlcTask::handleNEXT(const LadInstr& instr) {
    Q_UNUSED(instr);

    int nextIdx = _currCtx->pc;
    int loopIdx = _nextLoops[nextIdx];
    Q_ASSERT(loopIdx != -1);

    if (_loopCounts[loopIdx] > 1) {
        _loopCounts[loopIdx]--;
        _currCtx->pc = loopIdx;
        return;
    }

    _loopCounts[loopIdx] = 0;
}

void PlcTask::handleADD(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 + val2);
    }
}

void PlcTask::handleSUB(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 - val2);
    }
}

void PlcTask::handleMUL(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 * val2);
    }
}

void PlcTask::handleDIV(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 != 0) {
            setValue(_ladData.regs[instr.regIdx + 2], val1 / val2);
        } else {
            replaceLogicTop(false);
        }
    }
}

void PlcTask::handleMOD(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 != 0) {
            setValue(_ladData.regs[instr.regIdx + 2], val1 % val2);
        } else {
            replaceLogicTop(false);
        }
    }
}

void PlcTask::handleINC(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx], val1 + 1);
    }
}

void PlcTask::handleDEC(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx], val1 - 1);
    }
}

void PlcTask::handleABS(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], abs(val1));
    }
}

void PlcTask::handleNEG(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], -val1);
    }
}

void PlcTask::handleEQ(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 == val2);
    }
}

void PlcTask::handleNEQ(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 != val2);
    }
}

void PlcTask::handleGT(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 > val2);
    }
}

void PlcTask::handleLT(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 < val2);
    }
}

void PlcTask::handleGTE(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 >= val2);
    }
}

void PlcTask::handleLTE(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        replaceLogicTop(val1 <= val2);
    }
}

void PlcTask::handleRNG(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        int val3 = getValue(_ladData.regs[instr.regIdx + 2]);
        replaceLogicTop((val1 <= val3 && val3 <= val2) || (val2 <= val3 && val3 <= val1));
    }
}

void PlcTask::handleXOR(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 ^ val2);  
    }
}

void PlcTask::handleWAND(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 & val2);  
    }
}

void PlcTask::handleWOR(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 | val2);  
    }
}

void PlcTask::handleNOT(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], ~val1);  
    }
}

void PlcTask::handleSHL(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 > 0) {
            switch (_ladData.regs[instr.regIdx].type) {
                case REG_D:
                case REG_B:
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<int>(static_cast<unsigned int>(val1) << val2));
                    break;
                default:
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<char>(static_cast<unsigned char>(val1) << val2));
                    break;
            }
        } else {
            replaceLogicTop(false);
        }
    }
}

void PlcTask::handleSHR(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 > 0) {
            switch (_ladData.regs[instr.regIdx].type) {
                case REG_D:
                case REG_B:
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<int>(static_cast<unsigned int>(val1) >> val2));
                    break;
                default:
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<char>(static_cast<unsigned char>(val1) >> val2));
                    break;
            }
        } else {
            replaceLogicTop(false);
        }
    }
}

void PlcTask::handleROL(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 > 0) {
            switch (_ladData.regs[instr.regIdx].type) {
                case REG_D:
                case REG_B: {
                    unsigned int uval1 = static_cast<unsigned int>(val1);
                    unsigned int shift = static_cast<unsigned int>(val2) % 32;
                    // 避免 shift 为 0 时产生移位 32 位的未定义行为
                    unsigned int result = (shift == 0) ? uval1 : ((uval1 << shift) | (uval1 >> (32 - shift)));
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<int>(result));
                    break;
                }
                default: {
                    unsigned char uval1 = static_cast<unsigned char>(val1);
                    unsigned int shift = static_cast<unsigned int>(val2) % 8;
                    // 避免 shift 为 0 时产生移位 8 位的未定义行为
                    unsigned char result = (shift == 0) ? uval1 : static_cast<unsigned char>((uval1 << shift) | (uval1 >> (8 - shift)));
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<char>(result));
                    break;
                }
            }
        } else {
            replaceLogicTop(false);
        }
    }
}
void PlcTask::handleROR(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 > 0) {
            switch (_ladData.regs[instr.regIdx].type) {
                case REG_D:
                case REG_B: {
                    unsigned int uval1 = static_cast<unsigned int>(val1);
                    unsigned int shift = static_cast<unsigned int>(val2) % 32;
                    // 避免 shift 为 0 时产生移位 32 位的未定义行为
                    unsigned int result = (shift == 0) ? uval1 : ((uval1 >> shift) | (uval1 << (32 - shift)));
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<int>(result));
                    break;
                }
                default: {
                    unsigned char uval1 = static_cast<unsigned char>(val1);
                    unsigned int shift = static_cast<unsigned int>(val2) % 8;
                    // 避免 shift 为 0 时产生移位 8 位的未定义行为
                    unsigned char result = (shift == 0) ? uval1 : static_cast<unsigned char>((uval1 >> shift) | (uval1 << (8 - shift)));
                    setValue(_ladData.regs[instr.regIdx + 2], static_cast<char>(result));
                    break;
                }
            }
        } else {
            replaceLogicTop(false);
        }
    }
}

void PlcTask::handleTMRB(const LadInstr& instr) {
    int timerId = getValue(_ladData.regs[instr.regIdx]);
    int unit = getValue(_ladData.regs[instr.regIdx + 1]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 2]);

    int64_t presetMs = presetVal;
    if (unit == 1)      presetMs *= 1000LL;    // 秒
    else if (unit == 2) presetMs *= 60000LL;   // 分钟
    else if (unit == 3) presetMs *= 3600000LL; // 小时
    _regData.regT[timerId + REG_T_NUM] = static_cast<int>(std::clamp<int64_t>(presetMs, 0, INT_MAX));

    if (logicTop()) {
        if (_regData.regT[timerId] + timers[timerId] >= presetMs) {
            _regData.regT[timerId] = presetMs;
        } else {
            _regData.regT[timerId] += timers[timerId];
            replaceLogicTop(false);
        }
    } else {
        _regData.regT[timerId] = 0;
        replaceLogicTop(false);
    }
    timers[timerId] = 0;
}

void PlcTask::handleSTMR(const LadInstr& instr) {
    int timerId = getValue(_ladData.regs[instr.regIdx]);
    int unit = getValue(_ladData.regs[instr.regIdx + 1]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 2]);

    int64_t presetMs = presetVal;
    if (unit == 1)      presetMs *= 1000LL;    // 秒
    else if (unit == 2) presetMs *= 60000LL;   // 分钟
    else if (unit == 3) presetMs *= 3600000LL; // 小时
    _regData.regT[timerId + REG_T_NUM] = static_cast<int>(std::clamp<int64_t>(presetMs, 0, INT_MAX));

    if (logicTop()) {
        _regData.regT[timerId] = 0;
    } else {
        if (_regData.regT[timerId] + timers[timerId] >= presetMs) {
            _regData.regT[timerId] = presetMs;
        } else {
            _regData.regT[timerId] += timers[timerId];
            replaceLogicTop(true);
        }
    }
    timers[timerId] = 0;
}

void PlcTask::handleCTR(const LadInstr& instr) {
    CtrInputs inputs = popCtrInputs();
    bool actPulse = updateFlowRisingEdge(_currCtx->pc, inputs.act);

    int currentVal = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);
    int beginVal = inputs.cn0 ? 1 : 0;

    if (actPulse) {
        if (!inputs.upd && currentVal < presetVal) {
            setValue(_ladData.regs[instr.regIdx], currentVal + 1);
        }
        
        if (inputs.upd && currentVal > beginVal) {
            setValue(_ladData.regs[instr.regIdx], currentVal - 1);
        }
    }

    if (inputs.rst) {
        if (inputs.upd) {
            setValue(_ladData.regs[instr.regIdx], presetVal);
        } else {
            setValue(_ladData.regs[instr.regIdx], beginVal);
        }
    }

    if (inputs.upd) {
        replaceLogicTop(currentVal <= beginVal);
    } else {
        replaceLogicTop(currentVal >= presetVal);
    }
}

void PlcTask::handleCTRC(const LadInstr& instr) {
    CtrcInputs inputs = popCtrcInputs();
    bool actPulse = updateFlowRisingEdge(_currCtx->pc, inputs.act);

    int counterId = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);
    _regData.regC[counterId+ REG_C_NUM] = presetVal;

    if (actPulse) {
        if (_regData.regC[counterId] < presetVal) {
            _regData.regC[counterId]++;
        }
    }

    if (inputs.rst) {
        _regData.regC[counterId] = 0;
    }

    replaceLogicTop(_regData.regC[counterId] >= presetVal);
}

void PlcTask::handleCTUD(const LadInstr& instr) {
    CtrInputs inputs = popCtrInputs();
    bool actPulse = updateFlowRisingEdge(_currCtx->pc, inputs.act);

    int counterId = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);
    int beginVal = inputs.cn0 ? 1 : 0;
    _regData.regC[counterId+ REG_C_NUM] = presetVal;

    if (actPulse) {
        if (!inputs.upd && _regData.regC[counterId] < presetVal) {
            _regData.regC[counterId]++;
        }
        
        if (inputs.upd && _regData.regC[counterId] > beginVal) {
            _regData.regC[counterId]--;
        }
    }
    
    if (inputs.rst) {
        if (inputs.upd) {
            _regData.regC[counterId] = presetVal;
        } else {
            _regData.regC[counterId] = beginVal;
        }
    }

    if (inputs.upd) {
        replaceLogicTop(_regData.regC[counterId] <= beginVal);
    } else {
        replaceLogicTop(_regData.regC[counterId] >= presetVal);
    }
}

void PlcTask::handleMOV(const LadInstr& instr) {
    if (logicTop()) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], val1);
    }
}

void PlcTask::handleXMOV(const LadInstr& instr) {
    if (logicTop()) {
        LadReg& srcReg = _ladData.regs[instr.regIdx];
        int srcOffset = getValue(_ladData.regs[instr.regIdx + 1]);
        LadReg& dstReg = _ladData.regs[instr.regIdx + 2];
        int dstOffset = getValue(_ladData.regs[instr.regIdx + 3]);

        if (!isRegRangeValid(srcReg, srcOffset, 1) || !isRegRangeValid(dstReg, dstOffset, 1)) {
            replaceLogicTop(false);
            return;
        }

        LadReg src = srcReg;
        LadReg dst = dstReg;
        src.idx += srcOffset;
        dst.idx += dstOffset;
        setValue(dst, getValue(src));
        replaceLogicTop(true);
    }
}

void PlcTask::handleBMOV(const LadInstr& instr) {
    if (logicTop()) {
        int count = getValue(_ladData.regs[instr.regIdx]);
        LadReg& srcReg = _ladData.regs[instr.regIdx + 1];
        LadReg& dstReg = _ladData.regs[instr.regIdx + 2];

        if (!isRegRangeValid(srcReg, 0, count) || !isRegRangeValid(dstReg, 0, count)) {
            replaceLogicTop(false);
            return;
        }

        LadReg src = srcReg;
        LadReg dst = dstReg;
        for (int i = 0; i < count; i++, src.idx++, dst.idx++) {
            setValue(dst, getValue(src));
        }
        replaceLogicTop(true);
    }
}

void PlcTask::handleFMOV(const LadInstr& instr) {
    if (logicTop()) {
        int value = getValue(_ladData.regs[instr.regIdx]);
        LadReg& dstReg = _ladData.regs[instr.regIdx + 1];
        int count = getValue(_ladData.regs[instr.regIdx + 2]);

        if (!isRegRangeValid(dstReg, 0, count)) {
            replaceLogicTop(false);
            return;
        }

        LadReg dst = dstReg;
        for (int i = 0; i < count; i++, dst.idx++) {
            setValue(dst, value);
        }
        replaceLogicTop(true);
    }
}

void PlcTask::handleSER(const LadInstr& instr) {
    if (logicTop()) {
        LadReg& srcReg = _ladData.regs[instr.regIdx];
        int range = getValue(_ladData.regs[instr.regIdx + 1]);
        int target = getValue(_ladData.regs[instr.regIdx + 2]);
        LadReg& resultReg = _ladData.regs[instr.regIdx + 3];

        if (!isRegRangeValid(srcReg, 0, range)) {
            replaceLogicTop(false);
            return;
        }

        LadReg src = srcReg;
        for (int i = 0; i < range; i++, src.idx++) {
            if (getValue(src) == target) {
                setValue(resultReg, i);
                replaceLogicTop(true);
                return;
            }
        }

        replaceLogicTop(false);
    }
}

void PlcTask::handleASSEM(const LadInstr& instr) {
    if (logicTop()) {
        LadReg& srcReg = _ladData.regs[instr.regIdx];
        LadReg& dstReg = _ladData.regs[instr.regIdx + 1];
        uint32_t value = 0;

        if (!isRegRangeValid(srcReg, 0, 4) || !isRegRangeValid(dstReg, 0, 1)) {
            replaceLogicTop(false);
            return;
        }

        for (int i = 0; i < 4; i++) {
            value |= static_cast<uint32_t>(getByteValue(srcReg, i)) << (i * 8);
        }

        setWordValue(dstReg, value);
        replaceLogicTop(true);
    }
}

void PlcTask::handleDISAS(const LadInstr& instr) {
    if (logicTop()) {
        LadReg& srcReg = _ladData.regs[instr.regIdx];
        LadReg& dstReg = _ladData.regs[instr.regIdx + 1];

        if (!isRegRangeValid(srcReg, 0, 1) || !isRegRangeValid(dstReg, 0, 4)) {
            replaceLogicTop(false);
            return;
        }

        uint32_t value = getWordValue(srcReg);
        for (int i = 0; i < 4; i++) {
            setByteValue(dstReg, i, static_cast<unsigned char>((value >> (i * 8)) & 0xFFu));
        }

        replaceLogicTop(true);
    }
}

void PlcTask::handleALT(const LadInstr& instr) {
    Q_UNUSED(instr)

    bool currState = logicTop();
    bool actPulse = updateFlowRisingEdge(_currCtx->pc, currState);
    if (actPulse) {
        _altStates[_currCtx->pc] = !_altStates[_currCtx->pc];
    }
    replaceLogicTop(static_cast<bool>(_altStates[_currCtx->pc]));
}

void PlcTask::handleROT(const LadInstr& instr) {
    RotInputs inputs = popRotInputs();

    int rotorNum = getValue(_ladData.regs[instr.regIdx]);
    int currPos = getValue(_ladData.regs[instr.regIdx + 1]);
    int goalPos = getValue(_ladData.regs[instr.regIdx + 2]);
    int result = -1;

    if (inputs.act) {
        if (rotorNum <= 0) {
            return;
        }
        
        // 将位置全部统一转化为 0-based 索引以便取模计算
        if (inputs.rn0) {
            currPos -= 1;
            goalPos -= 1;
        }

        // 安全归一化到 [0, rotorNum - 1] 区间，防止异常输入
        currPos = (currPos) % rotorNum;
        goalPos = (goalPos % rotorNum + rotorNum) % rotorNum;

        // 计算正转(增加)和反转(减小)的距离步数
        int forDist = (goalPos - currPos + rotorNum) % rotorNum;
        int revDist = (currPos - goalPos + rotorNum) % rotorNum;

        // 决定旋转方向：isRev = true 为反转 (W1 = 1)
        bool isRev = false;
        if (inputs.dir) { // DIR=1: 启用最短路径选择
            if (revDist < forDist) {
                isRev = true;
            }
        }

        // 根据 POS 条件确定最终要计算的目标位置
        int targetPos0 = goalPos;
        if (inputs.pos) { // POS=1: 计算目标位置的前一位置
            if (isRev) {
                // 反转时，目标前一位置比目标位置索引多 1
                targetPos0 = (goalPos + 1) % rotorNum;
            } else {
                // 正转时，目标前一位置比目标位置索引少 1
                targetPos0 = (goalPos - 1 + rotorNum) % rotorNum;
            }
        }

        // 计算从当前位置到 targetPos0 需要走的步数
        int steps = isRev ? (currPos - targetPos0 + rotorNum) % rotorNum 
                            : (targetPos0 - currPos + rotorNum) % rotorNum;

        // 根据 INC 条件决定输出的是步数还是位置
        if (inputs.inc) {
            result = steps;                      // INC=1: 输出所需步数
        } else {
            result = targetPos0 + (inputs.rn0 ? 1 : 0); // INC=0: 输出目标位置 (如果是 1-based 则加回 1)
        }

        setValue(_ladData.regs[instr.regIdx + 3], result);

        // W1 = 0: 正向 (FOR), W1 = 1: 反向 (REV)
        replaceLogicTop(isRev);
        
    }
}

void PlcTask::setBit(LadReg& reg, bool value) {
    if (value) {
        setBit(reg);
    } else {
        clearBit(reg);
    }
}

void PlcTask::setBit(LadReg& reg) {
    switch (reg.type) {
        case REG_A:
            _regData.regA[reg.idx] = (char)((unsigned char)_regData.regA[reg.idx] | (1u << reg.bit));
            break;
        case REG_R:
            _regData.regR[reg.idx] = (char)((unsigned char)_regData.regR[reg.idx] | (1u << reg.bit));
            break;
        case REG_D:
            _regData.regD[reg.idx] = (int)((unsigned int)_regData.regD[reg.idx] | (1u << reg.bit));
            break;
        case REG_X:
            _regData.regX[reg.idx] = (char)((unsigned char)_regData.regX[reg.idx] | (1u << reg.bit));
            break;
        case REG_Y:
            _regData.regY[reg.idx] = (char)((unsigned char)_regData.regY[reg.idx] | (1u << reg.bit));
            break;
        case REG_F:
            _regData.regF[reg.idx] = (char)((unsigned char)_regData.regF[reg.idx] | (1u << reg.bit));
            break;
        case REG_G:
            _regData.regG[reg.idx] = (char)((unsigned char)_regData.regG[reg.idx] | (1u << reg.bit));
            break;
        case REG_I:
            _regData.regI[reg.idx] = (char)((unsigned char)_regData.regI[reg.idx] | (1u << reg.bit));
            break;
        case REG_Q:
            _regData.regQ[reg.idx] = (char)((unsigned char)_regData.regQ[reg.idx] | (1u << reg.bit));
            break;
        case REG_B:
            _regData.regB[reg.idx] = (int)((unsigned int)_regData.regB[reg.idx] | (1u << reg.bit));
            break;
        case REG_K:
            _regData.regK[reg.idx] = (char)((unsigned char)_regData.regK[reg.idx] | (1u << reg.bit));
            break;
        default:
            break;
    }
}

void PlcTask::clearBit(LadReg& reg) {
    switch (reg.type) {
        case REG_A:
            _regData.regA[reg.idx] = (char)((unsigned char)_regData.regA[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_R:
            _regData.regR[reg.idx] = (char)((unsigned char)_regData.regR[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_D:
            _regData.regD[reg.idx] = (int)((unsigned int)_regData.regD[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_X:
             _regData.regX[reg.idx] = (char)((unsigned char)_regData.regX[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_Y:
            _regData.regY[reg.idx] = (char)((unsigned char)_regData.regY[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_F:
            _regData.regF[reg.idx] = (char)((unsigned char)_regData.regF[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_G:
            _regData.regG[reg.idx] = (char)((unsigned char)_regData.regG[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_I:
            _regData.regI[reg.idx] = (char)((unsigned char)_regData.regI[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_Q: 
            _regData.regQ[reg.idx] = (char)((unsigned char)_regData.regQ[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_B:
            _regData.regB[reg.idx] = (int)((unsigned int)_regData.regB[reg.idx] & ~(1u << reg.bit));
            break;
        case REG_K:
            _regData.regK[reg.idx] = (char)((unsigned char)_regData.regK[reg.idx] & ~(1u << reg.bit));
            break;
        default:
            break;
    }
}

bool PlcTask::getBit(LadReg& reg) {
    switch (reg.type) {
        case REG_A:
            return (_regData.regA[reg.idx] >> reg.bit) & 1;
        case REG_R:
            return (_regData.regR[reg.idx] >> reg.bit) & 1;
        case REG_D:
            return (_regData.regD[reg.idx] >> reg.bit) & 1;
        case REG_X:
            return (_regData.regX[reg.idx] >> reg.bit) & 1;
        case REG_Y:
            return (_regData.regY[reg.idx] >> reg.bit) & 1;
        case REG_F:
            return (_regData.regF[reg.idx] >> reg.bit) & 1;
        case REG_G:
            return (_regData.regG[reg.idx] >> reg.bit) & 1;
        case REG_I:
            return (_regData.regI[reg.idx] >> reg.bit) & 1;
        case REG_Q:
            return (_regData.regQ[reg.idx] >> reg.bit) & 1;
        case REG_B:
            return (_regData.regB[reg.idx] >> reg.bit) & 1;
        case REG_K:
            return (_regData.regK[reg.idx] >> reg.bit) & 1;
        default:
            return false;
    }
}

int PlcTask::getValue(LadReg& reg) {
    switch (reg.type) {
        case REG_A:
            return _regData.regA[reg.idx];
        case REG_R:
            return _regData.regR[reg.idx];
        case REG_D:
            return _regData.regD[reg.idx];
        case REG_X:
            return _regData.regX[reg.idx];
        case REG_Y:
            return _regData.regY[reg.idx];
        case REG_F:
            return _regData.regF[reg.idx];
        case REG_G:
            return _regData.regG[reg.idx];
        case REG_I:
            return _regData.regI[reg.idx];
        case REG_Q:
            return _regData.regQ[reg.idx];
        case REG_B:
            return _regData.regB[reg.idx];
        case REG_K:
            return _regData.regK[reg.idx];
        case REG_TYPE_NONE:
            return reg.idx;
        default:
            return 0;
    }
}

void PlcTask::setValue(LadReg& reg, int value) {
    switch (reg.type) {
        case REG_A:
            _regData.regA[reg.idx] = value;
            break;
        case REG_R:
            _regData.regR[reg.idx] = value;
            break;
        case REG_D:
            _regData.regD[reg.idx] = value;
            break;
        case REG_X:
            _regData.regX[reg.idx] = value;
            break;
        case REG_Y:
            _regData.regY[reg.idx] = value;
            break;
        case REG_F:
            _regData.regF[reg.idx] = value;
            break;
        case REG_G:
            _regData.regG[reg.idx] = value;
            break;
        case REG_I:
            _regData.regI[reg.idx] = value;
            break;
        case REG_Q:
            _regData.regQ[reg.idx] = value;
            break;
        case REG_B:
            _regData.regB[reg.idx] = value;
            break;
        case REG_K:
            _regData.regK[reg.idx] = value;
            break;
        default:
            break;
    }
}

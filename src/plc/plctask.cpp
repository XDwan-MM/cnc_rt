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
    // 旋转操作
    _dispatchTable[OP_ROT] = &PlcTask::handleROT;
}

void PlcTask::resetContext() {
    _edgeStates.clear();
    _edgeIdxs.assign(_ladData.info.instrCount, false);
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
                _edgeStates.emplace_back(bitVal, bitVal, i);
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
            default:
                break;
        }
    }

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
            int& regIdx = _ladData.instrs[_edgeStates[i].instrIdx].regIdx;
            _edgeStates[i].currState = getBit(_ladData.regs[regIdx]);
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
            int& regIdx = _ladData.instrs[_edgeStates[i].instrIdx].regIdx;
            _edgeStates[i].currState = getBit(_ladData.regs[regIdx]);
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
    _currCtx->_logicStack <<= 1;
    _currCtx->_logicStack |= 1;
}

void PlcTask::handleLD(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    _currCtx->_logicStack <<= 1;
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleLDI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    _currCtx->_logicStack <<= 1;
    if (!bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleAND(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    if (bitVal) {
        // _currContext->_logicStack &= 0xFFFFFFFF;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleANI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    if (!bitVal) {
        //_currContext->_logicStack &= 0xFFFFFFFF;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleOR(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    }
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        // _currContext->_logicStack |= 0;
    }
}

void PlcTask::handleORI(const LadInstr& instr) {
    bool bitVal = getBit(_ladData.regs[instr.regIdx]);
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    }
    if (!bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        // _currContext->_logicStack |= 0;
    }
}

void PlcTask::handleLDP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (!last && curr) {
            bitVal = true; // 上升沿
        }
    }
    _currCtx->_logicStack <<= 1;
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleLDF(const LadInstr& instr) {
    bool bitVal = true;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (last && !curr) {
            bitVal = false; // 下降沿
        }
    }
    _currCtx->_logicStack <<= 1;
    if (!bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleANDP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (!last && curr) {
            bitVal = true; // 上升沿
        }
    }
    if (bitVal) {
        // _currContext->_logicStack &= 0xFFFFFFFF;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleANDF(const LadInstr& instr) {
    bool bitVal = true;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (last && !curr) {
            bitVal = false; // 下降沿
        }
    }
    if (!bitVal) {
        // _currContext->_logicStack &= 0xFFFFFFFF;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleORP(const LadInstr& instr) {
    bool bitVal = false;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = true;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = false;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (!last && curr) {
            bitVal = true; // 上升沿
        }
    }
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        // _currContext->_logicStack |= 0;
    }
}

void PlcTask::handleORF(const LadInstr& instr) {
    bool bitVal = true;
    if (instr.state == LAD_FORCE_ENABLE) {
        bitVal = false;
    } else if (instr.state == LAD_FORCE_DISABLE) {
        bitVal = true;
    } else {
        bool& last = _edgeStates[_edgeIdxs[_currCtx->pc]].lastState;
        bool& curr = _edgeStates[_edgeIdxs[_currCtx->pc]].currState;
        if (last && !curr) {
            bitVal = false; // 下降沿
        }
    }
    if (!bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        // _currContext->_logicStack |= 0;
    }
}

void PlcTask::handleOUT(const LadInstr& instr) {
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    setBit(_ladData.regs[instr.regIdx], bitVal);
}

void PlcTask::handleOOUT(const LadInstr& instr) {
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    setBit(_ladData.regs[instr.regIdx], !bitVal);
}

void PlcTask::handleSET(const LadInstr& instr) {
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    if (bitVal) {
        setBit(_ladData.regs[instr.regIdx]);
    }
}

void PlcTask::handleRST(const LadInstr& instr) {
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    if (bitVal) {
        clearBit(_ladData.regs[instr.regIdx]);
    }
}

void PlcTask::handleANB(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    if (bitVal) {
        // _currContext->_logicStack &= 0xFFFFFFFF;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleORB(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        // _currContext->_logicStack |= 0;
    }
}

void PlcTask::handleMPS(const LadInstr& instr) {
    Q_UNUSED(instr);
    bool bitVal = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack <<= 1;
    if (bitVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleMRD(const LadInstr& instr) {
    Q_UNUSED(instr);
    // 获取第 2 个 bit 的值
    uint32_t bitVal = (_currCtx->_logicStack >> 1) & 1; 
    // 置 0 第 1 个bit
    _currCtx->_logicStack &= ~1;
    _currCtx->_logicStack |= bitVal;
}

void PlcTask::handleMPP(const LadInstr& instr) {
    Q_UNUSED(instr);
    _currCtx->_logicStack >>= 1;
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
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
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
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        _currCtx->pc = _currCtx->_callStack.top();
        _currCtx->_callStack.pop();
    }
}

void PlcTask::handleLBL(const LadInstr& instr) {
    Q_UNUSED(instr);
}

void PlcTask::handleJMP(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        Q_ASSERT(_labelPositions[_ladData.regs[instr.regIdx].idx] != -1);
        // 跳转到标签位置
        _currCtx->pc = _labelPositions[_ladData.regs[instr.regIdx].idx];
    }
}

void PlcTask::handleADD(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 + val2);
    }
}

void PlcTask::handleSUB(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 - val2);
    }
}

void PlcTask::handleMUL(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 * val2);
    }
}

void PlcTask::handleDIV(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 != 0) {
            setValue(_ladData.regs[instr.regIdx + 2], val1 / val2);
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleMOD(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val2 != 0) {
            setValue(_ladData.regs[instr.regIdx + 2], val1 % val2);
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleINC(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx], val1 + 1);
    }
}

void PlcTask::handleDEC(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx], val1 - 1);
    }
}

void PlcTask::handleABS(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], abs(val1));
    }
}

void PlcTask::handleNEG(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], -val1);
    }
}

void PlcTask::handleEQ(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 == val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleNEQ(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 != val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleGT(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 > val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleLT(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 < val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleGTE(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 >= val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleLTE(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        if (val1 <= val2) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleRNG(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        int val3 = getValue(_ladData.regs[instr.regIdx + 2]);
        if ((val1 <= val3 && val3 <= val2) || (val2 <= val3 && val3 <= val1)) {
            // _currContext->_logicStack &= 0xFFFFFFFF;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleXOR(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 ^ val2);  
    }
}

void PlcTask::handleWAND(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 & val2);  
    }
}

void PlcTask::handleWOR(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        int val2 = getValue(_ladData.regs[instr.regIdx + 1]);
        setValue(_ladData.regs[instr.regIdx + 2], val1 | val2);  
    }
}

void PlcTask::handleNOT(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], ~val1);  
    }
}

void PlcTask::handleSHL(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
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
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleSHR(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
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
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleROL(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
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
            _currCtx->_logicStack &= ~1;
        }
    }
}
void PlcTask::handleROR(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
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
            _currCtx->_logicStack &= ~1;
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

    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        if (_regData.regT[timerId] + timers[timerId] >= presetMs) {
            _regData.regT[timerId] = presetMs;
        } else {
            _regData.regT[timerId] += timers[timerId];
            _currCtx->_logicStack &= ~1;
        }
    } else {
        _regData.regT[timerId] = 0;
        _currCtx->_logicStack &= ~1;
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

    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        _regData.regT[timerId] = 0;
    } else {
        if (_regData.regT[timerId] + timers[timerId] >= presetMs) {
            _regData.regT[timerId] = presetMs;
        } else {
            _regData.regT[timerId] += timers[timerId];
            _currCtx->_logicStack |= 1;
        }
    }
    timers[timerId] = 0;
}

void PlcTask::handleCTR(const LadInstr& instr) {
    bool rst = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool upd = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool cn0 = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool act = static_cast<bool>(_currCtx->_logicStack & 1);

    int currentVal = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);

    if (act) {
        if (!upd && currentVal < presetVal) {
            setValue(_ladData.regs[instr.regIdx], currentVal + 1);
        }
        
        if (upd &&  currentVal > cn0 ? 1 : 0) {
            setValue(_ladData.regs[instr.regIdx], currentVal - 1);
        }
    }

    if (rst) {
        if (upd) {
            setValue(_ladData.regs[instr.regIdx], presetVal);
        } else {
            setValue(_ladData.regs[instr.regIdx], cn0 ? 1 : 0);
        }
    }

    if (upd) {
        if (currentVal <= cn0 ? 1 : 0) {
            _currCtx->_logicStack |= 1;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    } else {
        if (currentVal >= presetVal)  {
            _currCtx->_logicStack |= 1;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleCTRC(const LadInstr& instr) {
    bool rst = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool act = static_cast<bool>(_currCtx->_logicStack & 1);

    int counterId = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);
    _regData.regC[counterId+ REG_C_NUM] = presetVal;

    if (act) {
        if (_regData.regC[counterId] < presetVal) {
            _regData.regC[counterId]++;
        }
    }

    if (rst) {
        _regData.regC[counterId] = 0;
    }

    if (_regData.regC[counterId] >= presetVal) {
        _currCtx->_logicStack |= 1;
    } else {
        _currCtx->_logicStack &= ~1;
    }
}

void PlcTask::handleCTUD(const LadInstr& instr) {
    bool rst = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool upd = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool cn0 = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool act = static_cast<bool>(_currCtx->_logicStack & 1);

    int counterId = getValue(_ladData.regs[instr.regIdx]);
    int presetVal = getValue(_ladData.regs[instr.regIdx + 1]);
    _regData.regC[counterId+ REG_C_NUM] = presetVal;

    if (act) {
        if (!upd && _regData.regC[counterId] < presetVal) {
            _regData.regC[counterId]++;
        }
        
        if (upd &&  _regData.regC[counterId] > cn0 ? 1 : 0) {
            _regData.regC[counterId]--;
        }
    }
    
    if (rst) {
        if (upd) {
            _regData.regC[counterId] = presetVal;
        } else {
            _regData.regC[counterId] = cn0 ? 1 : 0;
        }
    }

    if (upd) {
        if (_regData.regC[counterId] <= cn0 ? 1 : 0) {
            _currCtx->_logicStack |= 1;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    } else {
        if (_regData.regC[counterId] >= presetVal)  {
            _currCtx->_logicStack |= 1;
        } else {
            _currCtx->_logicStack &= ~1;
        }
    }
}

void PlcTask::handleMOV(const LadInstr& instr) {
    if (static_cast<bool>(_currCtx->_logicStack & 1)) {
        int val1 = getValue(_ladData.regs[instr.regIdx]);
        setValue(_ladData.regs[instr.regIdx + 1], val1);
    }
}

void PlcTask::handleROT(const LadInstr& instr) {
    bool inc = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool pos = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool dir = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool rn0 = static_cast<bool>(_currCtx->_logicStack & 1);
    _currCtx->_logicStack >>= 1;
    bool act = static_cast<bool>(_currCtx->_logicStack & 1);

    int rotorNum = getValue(_ladData.regs[instr.regIdx]);
    int currPos = getValue(_ladData.regs[instr.regIdx + 1]);
    int goalPos = getValue(_ladData.regs[instr.regIdx + 2]);
    int result = -1;

    if (act) {
        if (rotorNum <= 0) {
            return;
        }
        
        // 将位置全部统一转化为 0-based 索引以便取模计算
        if (rn0) {
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
        if (dir) { // DIR=1: 启用最短路径选择
            if (revDist < forDist) {
                isRev = true;
            }
        }

        // 根据 POS 条件确定最终要计算的目标位置
        int targetPos0 = goalPos;
        if (pos) { // POS=1: 计算目标位置的前一位置
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
        if (inc) {
            result = steps;                      // INC=1: 输出所需步数
        } else {
            result = targetPos0 + (rn0 ? 1 : 0); // INC=0: 输出目标位置 (如果是 1-based 则加回 1)
        }

        setValue(_ladData.regs[instr.regIdx + 3], result);

        // W1 = 0: 正向 (FOR), W1 = 1: 反向 (REV)
        if (isRev) {
            _currCtx->_logicStack |= 1;
        } else {
            _currCtx->_logicStack &= ~1;
        }
        
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
#include "ladservice.h"
#include "ladapi.h"
#include "laddata.h"
#include <QDebug>

LadService::LadService()
    : _data(*initializeLadData()) {
    FuncRectType rectType;
    rectType.hasLeft = true;
    rectType.hasRight = true;
    rectType.hasTop = true;
    rectType.hasBottom = true;
    _data.rows[0].cells[0].opcode = OP_IEND;
    _data.rows[0].cells[0].rectType = rectType;
    _data.rows[1].cells[0].opcode = OP_1END;
    _data.rows[1].cells[0].rectType = rectType;
    _data.rows[2].cells[0].opcode = OP_2END;
    _data.rows[2].cells[0].rectType = rectType;
    setTotalRowNum(3);
}

LadData* LadService::initializeLadData() {
    LadData* ptr = getLadDataPtr();
    if (ptr == nullptr) {
        // 如果API返回空指针，无法初始化引用，这是个严重错误
        // Todo：日志记录
        qDebug() << "共享内存打开失败，获取不到LadData指针";
    }
    return ptr;
}

const LadRow* LadService::getLadRowPtr(int row) {
    return &_data.rows[row];
}

int LadService::getTotalRowNum() {
    return _data.info.rowCount;
}

void LadService::setTotalRowNum(int totalRowNum) {
    _data.info.rowCount = totalRowNum;
}

void LadService::addCell(int row, int col, Opcode opcode) {
    switch (opcode) {
        case OP_VLINE:
            _data.rows[row].cells[col].hasVLine = true;
            break;
        case OP_LD:
        case OP_LDI:
        case OP_LDP:
        case OP_LDF:
        case OP_HLINE:
            _data.rows[row].cells[col].opcode = opcode;
            break;
        case OP_OUT:
        case OP_OOUT:
        case OP_SET:
        case OP_RST:
            _data.rows[row].cells[col].opcode = opcode;
            break;
        default:
            break;
    }
}

void LadService::deleteCell(int row, int col) {
    _data.rows[row].cells[col].hasParam = false;
    _data.rows[row].cells[col].opcode = OP_NONE;
}

void LadService::deleteVerticalLine(int row, int col) {
    _data.rows[row].cells[col].hasVLine = false;
}

void LadService::setCellContent(int row, int col, LadReg reg) {
    _data.rows[row].cells[col].hasParam = true;
    _data.rows[row].cells[col].reg = reg;
}

bool LadService::addRow(int row) {
    int totalRowNum = getTotalRowNum();
    if (totalRowNum == LAD_ROW_MAX_NUM) {
        qDebug() << "行数超出最大行数";
        return false;
    }
    for (int i = totalRowNum; i > row; i--) {
        _data.rows[i] = _data.rows[i - 1];
    }
    memset(&_data.rows[row], 0, sizeof(LadRow));
    setTotalRowNum(totalRowNum + 1);
    return true;
}

bool LadService::deleteRow(int row) {
    int totalRowNum = getTotalRowNum();
    if (totalRowNum == 0) {
        qDebug() << "行数为0";
        return false;
    }
    for (int i = row; i < totalRowNum - 1; i++) {
        _data.rows[i] = _data.rows[i + 1];
    }
    memset(&_data.rows[totalRowNum - 1], 0, sizeof(LadRow));
    setTotalRowNum(totalRowNum - 1);
    return true;
}

#include "ladviewdata.h"
#include "ladservice.h"
#include <QDebug>
#include <QRegularExpression>

LadViewData::LadViewData()
    : _service(LadService::instance()) {
}

int LadViewData::getVisualRowCount() {
    int rowCount = _service.getTotalRowNum();
    if (rowCount > cellPerCol) rowCount = cellPerCol;
    return rowCount;
}

void LadViewData::setCurrentRow(int row) {
    if (row < 0) row = 0;
    int totalRowNum = _service.getTotalRowNum();
    if (row >= totalRowNum) row = totalRowNum - 1;
    refreshCurrentSubprg();
    currRow = row;
    if (currRow < beginRow) beginRow = currRow;
    if (currRow > beginRow + cellPerCol - 1) beginRow = currRow - cellPerCol + 1;
}

void LadViewData::refreshCurrentSubprg() {
}

bool LadViewData::charToRegType(char c, RegType& type) {
    switch (c) {
        case 'R':
            type = REG_R;
            break;
        case 'W':
            type = REG_W;
            break;
        case 'D':
            type = REG_D;
            break;
        case 'X':
            type = REG_X;
            break;
        case 'Y':
            type = REG_Y;
            break;
        case 'F':
            type = REG_F;
            break;
        case 'G':
            type = REG_G;
            break;
        case 'I':
            type = REG_I;
            break;
        case 'Q':
            type = REG_Q;
            break;
        case 'K':
            type = REG_K;
            break;
        case 'B':
            type = REG_B;
            break;
        case 'T':
            type = REG_T;
            break;
        case 'C':
            type = REG_C;
            break;
        default:
            return false;
    }
    return true;
}

bool LadViewData::stringToLadReg(QString str, LadReg& reg) {
    // 正则表达式解析文本
    QRegularExpression regExp("^(X|Y|R|D|F|G|I|Q|K)([0-9]+)\\.([0-7])$");
    QRegularExpressionMatch match = regExp.match(str.toUpper());
    if (!match.hasMatch()) return false;
    char typeChar = match.captured(1).at(0).toLatin1();
    if (!charToRegType(typeChar, reg.type)) return false;
    reg.idx = match.captured(2).toInt();
    reg.bit = match.captured(3).toInt();
    return true;
}

bool LadViewData::opcodeToString(Opcode opcode, QString& str) {
    switch (opcode) {
        case OP_IEND:
            str = "IEND";
            break;
        case OP_1END:
            str = "1END";
            break;
        case OP_2END:
            str = "2END";
            break;
        default:
            return false;
    }
    return true;
}

#pragma once
#include "laddata.h"
#include <QString>
#include <algorithm>

class LadService;

class LadViewData {
public:
    LadViewData(const LadViewData&) = delete;
    LadViewData(LadViewData&&) = delete;
    LadViewData& operator=(const LadViewData&) = delete;
    LadViewData& operator=(LadViewData&&) = delete;
    static LadViewData& instance() {
        static LadViewData _instance;
        return _instance;
    }

    int currRow, currCol = 0;
    int beginRow = 0;
    static constexpr int cellPerRow = LAD_CELL_PER_ROW;
    static constexpr double verRatio = 1024.0 / 800.0;
    static constexpr double horRatio = 768.0 / 600.0;
    static constexpr double fontRatio = std::min(verRatio, horRatio);
    static constexpr int cellPerCol = 8;
    static constexpr double cellWidth = 75 * verRatio;
    static constexpr double cellHeight = 48 * horRatio;
    static constexpr double leftBlank = 30 * horRatio;

    int getVisualRowCount();
    void setCurrentRow(int row);
    void refreshCurrentSubprg();
    bool stringToLadReg(QString str, LadReg& reg);
    bool charToRegType(char c, RegType& type);
    bool opcodeToString(Opcode opcode, QString& str);

private:
    LadViewData();
    ~LadViewData() = default;

    LadService& _service;
};

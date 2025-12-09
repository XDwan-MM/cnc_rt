#pragma once
#include "laddata.h"
#include <QString>

struct LadRow;
struct LadData;

class LadService {
public:
    LadService(const LadService&) = delete;
    LadService(LadService&&) = delete;
    LadService& operator=(const LadService&) = delete;
    LadService& operator=(LadService&&) = delete;
    static LadService& instance() {
        static LadService _instance;
        return _instance;
    }

    const LadRow* getLadRowPtr(int row);
    int getTotalRowNum();
    void setTotalRowNum(int totalRowNum);
    // 编辑
    void addCell(int row, int col, Opcode opcode);
    void deleteCell(int row, int col);
    void deleteVerticalLine(int row, int col);
    void setCellContent(int row, int col, LadReg reg);
    bool addRow(int row);
    bool deleteRow(int row);

private:
    explicit LadService();
    ~LadService() = default;
    LadData& _data;
    static LadData* initializeLadData();
};

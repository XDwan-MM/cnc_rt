#include "ladview.h"
#include "eventsimulatordialog.h"
#include "ladservice.h"
#include "ladviewdata.h"
#include "ui_ladview.h"

#include <QRegularExpression>

LadView::LadView(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::LadView)
    , _data(LadViewData::instance())
    , _service(LadService::instance()) {
    ui->setupUi(this);

    EventSimulatorDialog* eventSimulatorDialog = new EventSimulatorDialog(this);
    eventSimulatorDialog->show();
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnLdClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_LD);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnLdiClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_LDI);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnOutClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_OUT);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnOoutClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_OOUT);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnHLineClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_HLINE);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnVLineClicked, this, [this]() {
        _service.addCell(_data.currRow, _data.currCol, OP_VLINE);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnDeleteItemClicked, this, [this]() {
        _service.deleteCell(_data.currRow, _data.currCol);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnDeleteVLineClicked, this, [this]() {
        _service.deleteVerticalLine(_data.currRow, _data.currCol);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::inputFinished, this, [this](const QString& text) {
        LadReg reg;
        if (!_data.stringToLadReg(text, reg)) return;
        _service.setCellContent(_data.currRow, _data.currCol, reg);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnAddRowClicked, this, [this]() {
        _service.addRow(_data.currRow);
        refresh();
    });
    connect(eventSimulatorDialog, &EventSimulatorDialog::btnDeleteRowClicked, this, [this]() {
        _service.deleteRow(_data.currRow);
        refresh();
    });

    connect(ui->verticalScrollBar, &QScrollBar::valueChanged, this, &LadView::onScrollBarChanged);
    connect(ui->ladPaintWidget, &LadPaintWidget::scrollRequest, this, &LadView::onScrollRequested);
    connect(ui->ladPaintWidget, &LadPaintWidget::currentCellChanged, this, &LadView::onCurrentCellChanged);

    refreshScrollBarMax();
}

LadView::~LadView() {
    delete ui;
}

void LadView::onScrollBarChanged(int value) {
    if (_data.beginRow != value) {
        _data.beginRow = value;
        // 通知绘图控件重绘
        ui->ladPaintWidget->update();
    }
}

void LadView::onScrollRequested(int numSteps) {
    int currentValue = ui->verticalScrollBar->value();
    setVerticalScrollBarVal(currentValue - numSteps);
}

void LadView::onCurrentCellChanged(int row, int col) {
    Q_UNUSED(row);
    Q_UNUSED(col);
    refresh();
}

void LadView::refreshScrollBarMax() {
    int totalRow = _service.getTotalRowNum();
    int visibleRows = _data.cellPerCol;
    int maxNum = (totalRow > visibleRows) ? (totalRow - visibleRows) : 0;

    if (ui->verticalScrollBar->maximum() != maxNum) {
        ui->verticalScrollBar->setMaximum(maxNum);
    }
}

void LadView::setVerticalScrollBarVal(int val) {
    refreshScrollBarMax();
    int maxNum = ui->verticalScrollBar->maximum();

    if (val > maxNum) {
        val = maxNum;
    } else if (val < 0) {
        val = 0;
    }

    if (ui->verticalScrollBar->value() != val) {
        ui->verticalScrollBar->setValue(val);
    }
}

void LadView::refresh() {
    ui->ladPaintWidget->update();
    // 行列字符串更新
    QString str = QString("行：%1/%2    列：%3/%4").arg(_data.currRow + 1).arg(_service.getTotalRowNum()).arg(_data.currCol + 1).arg(_data.cellPerRow);
    ui->lblPageInfo->setText(str);
    // 更新滚动条
    if (ui->verticalScrollBar->value() != _data.beginRow) {
        setVerticalScrollBarVal(_data.beginRow);
    }
}

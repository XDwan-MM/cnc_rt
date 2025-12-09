#pragma once
#include <QWidget>

namespace Ui {
class LadView;
}

struct LadRow;
struct LadCell;
class LadViewData;
class LadService;

class LadView : public QWidget {
    Q_OBJECT

public:
    enum class Mode {
        Edit,
        Monitor
    };
    explicit LadView(QWidget* parent = nullptr);
    ~LadView();
    Mode mode() { return _mode; }
    void setMode(Mode mode) { _mode = mode; }

private slots:
    void onScrollBarChanged(int value);
    void onScrollRequested(int numSteps);
    void onCurrentCellChanged(int row, int col);

private:
    Ui::LadView* ui;
    Mode _mode;
    LadViewData& _data;
    LadService& _service;
    void refreshScrollBarMax();
    void setVerticalScrollBarVal(int val);
    void refresh();
};

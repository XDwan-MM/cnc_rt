#pragma once
#include <QObject>
#include <QWidget>

struct LadRow;
struct LadCell;
class LadViewData;
class LadService;

class LadPaintWidget : public QWidget {
    Q_OBJECT
public:
    explicit LadPaintWidget(QWidget* parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    LadViewData& _data;
    LadService& _service;
    void drawRowNo(QPainter& painter, int y, int no);
    void drawRow(QPainter& painter, const LadRow& row, int y);
    void drawCell(QPainter& painter, const LadCell& cell, int x, int y);
    void drawFunction(QPainter& painter, const LadCell& cell, int x, int y);
    void drawFunctionName(QPainter& painter, const LadCell& cell, int x, int y);
    void drawCursor(QPainter& painter);
    void drawContact(QPainter& painter, int x, int y);
    void drawCoil(QPainter& painter, int x, int y);

signals:
    void scrollRequest(int val);
    void currentCellChanged(int row, int col);
};

#include "ladpaintwidget.h"
#include "ladservice.h"
#include "ladviewdata.h"
#include <QKeyEvent>
#include <QPainter>
#include <QPen>
#include <QWheelEvent>

LadPaintWidget::LadPaintWidget(QWidget* parent)
    : QWidget{parent}
    , _data(LadViewData::instance())
    , _service(LadService::instance()) {
    // 设置焦点策略，以便接收键盘事件
    setFocusPolicy(Qt::StrongFocus);
}

void LadPaintWidget::keyPressEvent(QKeyEvent* event) {
    int oldRow = _data.currRow;
    int oldCol = _data.currCol;
    switch (event->key()) {
        case Qt::Key_Left:
            if (_data.currCol > 0) {
                _data.currCol--;
            } else if (_data.currRow > 0) {
                _data.setCurrentRow(_data.currRow - 1);
                _data.currCol = _data.cellPerRow - 1;
            }
            break;
        case Qt::Key_Right:
            if (_data.currCol < _data.cellPerRow - 1) {
                _data.currCol++;
            } else if (_data.currRow < _service.getTotalRowNum() - 1) {
                _data.setCurrentRow(_data.currRow + 1);
                _data.currCol = 0;
            }
            break;
        case Qt::Key_Up:
            _data.setCurrentRow(_data.currRow - 1);
            break;
        case Qt::Key_Down:
            _data.setCurrentRow(_data.currRow + 1);
            break;
        case Qt::Key_Home:
            _data.currCol = 0;
            break;
        case Qt::Key_End:
            _data.currCol = _data.cellPerRow - 1;
            break;
        case Qt::Key_PageUp:
            _data.setCurrentRow(_data.currRow - _data.cellPerCol);
            break;
        case Qt::Key_PageDown:
            _data.setCurrentRow(_data.currRow + _data.cellPerCol);
            break;
        default:
            // 调用基类键盘事件
            QWidget::keyPressEvent(event);
    }
    if (oldRow != _data.currRow || oldCol != _data.currCol) {
        emit currentCellChanged(_data.currRow, _data.currCol);
        update();
    }
    event->accept();
}

void LadPaintWidget::drawCursor(QPainter& painter) {
    int left = _data.cellWidth * _data.currCol + _data.leftBlank;
    int top = _data.cellHeight * (_data.currRow - _data.beginRow);
    int right = left + _data.cellWidth;
    int bottom = top + _data.cellHeight;

    // painter.save();
    QPen pen(QColor(25, 115, 200));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(Qt::transparent);
    painter.drawRect(left, top, right - left, bottom - top);
    // painter.restore();
}

void LadPaintWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);

    // 设置字体
    QFont font = painter.font();
    font.setPointSizeF(font.pointSizeF() * _data.fontRatio);
    painter.setFont(font);

    // 绘制背景
    painter.setBrush(QBrush(QColor(255, 255, 255)));
    painter.drawRect(0, 0, width(), height());

    // 绘制梯形图左右母线
    painter.drawLine(_data.leftBlank, 0, _data.leftBlank, _data.cellHeight * _data.cellPerCol);
    painter.drawLine(_data.leftBlank + _data.cellPerRow * _data.cellWidth, 0, _data.leftBlank + _data.cellPerRow * _data.cellWidth, _data.cellHeight * _data.cellPerCol);

    int rowCount = _data.getVisualRowCount();
    for (int i = 0, yPos = 0; i < rowCount; i++) {
        const LadRow* row = _service.getLadRowPtr(i + _data.beginRow);
        drawRowNo(painter, yPos + i * _data.cellHeight, _data.beginRow + i + 1);
        drawRow(painter, *row, yPos + _data.cellHeight * i);
    }

    // 绘制焦点
    drawCursor(painter);
}

void LadPaintWidget::mousePressEvent(QMouseEvent* event) {
    // 调用基类鼠标事件
    if (event->button() != Qt::LeftButton) {
        return QWidget::mousePressEvent(event);
    }

    // 检查点击是否在有效范围内
    if (event->x() > _data.leftBlank && event->x() < _data.leftBlank + _data.cellWidth * _data.cellPerRow && event->y() > 0 && event->y() < _data.cellHeight * _data.cellPerCol) {
        int oldRow = _data.currRow;
        int oldCol = _data.currCol;
        int clickedCol = (event->x() - _data.leftBlank) / _data.cellWidth;
        int clickedRow = (event->y() / _data.cellHeight) + _data.beginRow;

        if ((oldRow != clickedRow || oldCol != clickedCol) && clickedRow >= _service.getTotalRowNum()) {
            _data.currCol = clickedCol;
            _data.currRow = clickedRow;
            emit currentCellChanged(_data.currRow, _data.currCol);
            update();
        }
    }
    event->accept();
}

void LadPaintWidget::wheelEvent(QWheelEvent* event) {
    QPoint angle = event->angleDelta();

    // 检查是否有垂直滚动的分量
    if (!angle.isNull() && angle.y() != 0) {
        // 直接用滚动的角度值除以 120 来计算步数
        int numSteps = angle.y() / 120;
        // 发出滚动请求信号，由父控件处理
        emit scrollRequest(numSteps);
        event->accept();
    } else {
        QWidget::wheelEvent(event);
    }
}

void LadPaintWidget::drawRowNo(QPainter& painter, int y, int no) {
    QRect qrect;
    qrect.setTop(y);
    qrect.setLeft(0);
    qrect.setBottom(y + _data.cellHeight);
    qrect.setRight(_data.leftBlank * 0.9);
    painter.drawText(qrect, Qt::AlignRight | Qt::AlignVCenter, QString::number(no));
}

void LadPaintWidget::drawRow(QPainter& painter, const LadRow& row, int y) {
    int xPos = _data.leftBlank;

    for (int i = 0; i < _data.cellPerRow; i++) {
        drawCell(painter, row.cells[i], xPos + i * _data.cellWidth, y);
    }
}

// 绘制触点
void LadPaintWidget::drawContact(QPainter& painter, int x, int y) {
    painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth / 3, y + _data.cellHeight / 2);
    painter.drawLine(x + _data.cellWidth / 3, y + _data.cellHeight * 10 / 27, x + _data.cellWidth / 3, y - _data.cellHeight * 10 / 27 + _data.cellHeight);
    painter.drawLine(x + 2 * _data.cellWidth / 3, y + _data.cellHeight * 10 / 27, x + 2 * _data.cellWidth / 3, y - _data.cellHeight * 10 / 27 + _data.cellHeight);
    painter.drawLine(x + 2 * _data.cellWidth / 3, y + _data.cellHeight / 2, x + _data.cellWidth, y + _data.cellHeight / 2);
}

// 绘制线圈
void LadPaintWidget::drawCoil(QPainter& painter, int x, int y) {
    painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth / 5, y + _data.cellHeight / 2);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.drawArc(x + _data.cellWidth / 5, y + _data.cellHeight / 4, _data.cellWidth * 2 / 5, _data.cellHeight / 2, 150 * 16, 60 * 16);
    painter.drawArc(x + _data.cellWidth * 2 / 5, y + _data.cellHeight / 4, _data.cellWidth * 2 / 5, _data.cellHeight / 2, -30 * 16, 60 * 16);
    painter.setRenderHint(QPainter::Antialiasing, false);
}

// 绘制单元格
void LadPaintWidget::drawCell(QPainter& painter, const LadCell& cell, int x, int y) {
    // 绘制竖线
    if (cell.hasVLine) {
        QRect qrect = this->geometry();
        if (y + 3 * _data.cellHeight / 2 < qrect.bottom()) {
            painter.drawLine(x + _data.cellWidth, y + _data.cellHeight / 2, x + _data.cellWidth, y + 3 * _data.cellHeight / 2);
        } else {
            painter.drawLine(x + _data.cellWidth, y + _data.cellHeight / 2, x + _data.cellWidth, qrect.bottom());
        }
    }

    // 绘制参数
    if (cell.hasParam) {
        QString text = QString("%1%2.%3").arg(static_cast<char>(cell.reg.type)).arg(cell.reg.idx).arg(QString::number(cell.reg.bit));
        switch (cell.opcode) {
            case OP_HLINE:
                break;
            case OP_LD:
            case OP_LDI:
            case OP_LDP:
            case OP_LDF:
            case OP_SET:
            case OP_RST:
            case OP_OUT:
            case OP_OOUT:
                painter.drawText(QRectF(x + _data.cellWidth / 4, y, _data.cellWidth / 2, _data.cellHeight / 4), Qt::AlignCenter, text);
                break;
            default: // 功能元件
                break;
        }
    }

    // 绘制元件
    switch (cell.opcode) {
        case OP_HLINE:
            painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth, y + _data.cellHeight / 2);
            break;
        case OP_LD:
            drawContact(painter, x, y);
            break;
        case OP_LDI:
            drawContact(painter, x, y);
            painter.drawLine(x + 4 * _data.cellWidth / 9, y + 7 * _data.cellHeight / 12, x + 5 * _data.cellWidth / 9, y + 5 * _data.cellHeight / 12);
            break;
        case OP_LDP:
            drawContact(painter, x, y);
            painter.drawLine(x + _data.cellWidth * 9 / 24, y + _data.cellHeight * 7 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 7 / 12);
            painter.drawLine(x + _data.cellWidth / 2, y + _data.cellHeight * 5 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 7 / 12);
            painter.drawLine(x + _data.cellWidth * 15 / 24, y + _data.cellHeight * 5 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 5 / 12);
            break;
        case OP_LDF:
            drawContact(painter, x, y);
            painter.drawLine(x + _data.cellWidth * 9 / 24, y + _data.cellHeight * 5 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 5 / 12);
            painter.drawLine(x + _data.cellWidth / 2, y + _data.cellHeight * 5 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 7 / 12);
            painter.drawLine(x + _data.cellWidth * 15 / 24, y + _data.cellHeight * 7 / 12, x + _data.cellWidth / 2, y + _data.cellHeight * 7 / 12);
            break;
        case OP_SET:
            drawCoil(painter, x, y);
            painter.drawText(QRectF(x + _data.cellWidth / 4, y + 3 * _data.cellHeight / 8, _data.cellWidth / 2, _data.cellHeight / 4), Qt::AlignCenter, "S");
            break;
        case OP_RST:
            drawCoil(painter, x, y);
            painter.drawText(QRectF(x + _data.cellWidth / 4, y + 3 * _data.cellHeight / 8, _data.cellWidth / 2, _data.cellHeight / 4), Qt::AlignCenter, "R");
            break;
        case OP_OUT:
            drawCoil(painter, x, y);
            break;
        case OP_OOUT:
            drawCoil(painter, x, y);
            painter.drawLine(x + 4 * _data.cellWidth / 9, y + 7 * _data.cellHeight / 12, x + 5 * _data.cellWidth / 9, y + 5 * _data.cellHeight / 12);
            break;
        default:
            drawFunction(painter, cell, x, y);
            break;
    }
}

// 绘制功能块
void LadPaintWidget::drawFunction(QPainter& painter, const LadCell& cell, int x, int y) {
    const bool hasLeft = cell.rectType.hasLeft;
    const bool hasRight = cell.rectType.hasRight;
    const bool hasTop = cell.rectType.hasTop;
    const bool hasBottom = cell.rectType.hasBottom;

    if (hasLeft && !hasRight && !hasTop && !hasBottom) {
        // 左
        painter.drawLine(x + _data.cellWidth / 6, y, x + _data.cellWidth / 6, y + _data.cellHeight);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y, x + _data.cellWidth * 5 / 6, y + _data.cellHeight);
    } else if (hasLeft && !hasRight && hasTop && !hasBottom) {
        // 左上
        drawFunctionName(painter, cell, x, y);
        painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth / 6, y + _data.cellHeight / 2);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth, y + _data.cellHeight / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth / 6, y + _data.cellHeight);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight);
    } else if (hasLeft && !hasRight && hasTop && hasBottom) {
        // 左上下
        drawFunctionName(painter, cell, x, y);
        painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth / 6, y + _data.cellHeight / 2);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth, y + _data.cellHeight / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6, x + _data.cellWidth, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
    } else if (hasLeft && !hasRight && !hasTop && hasBottom) {
        // 左下
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6, x + _data.cellWidth, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
    } else if (!hasLeft && hasRight && !hasTop && !hasBottom) {
        // 右
        painter.drawLine(x + _data.cellWidth * 5 / 6, y, x + _data.cellWidth * 5 / 6, y + _data.cellHeight);
    } else if (!hasLeft && hasRight && hasTop && !hasBottom) {
        // 右上
        painter.drawLine(x, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight);
    } else if (!hasLeft && hasRight && hasTop && hasBottom) {
        // 右上下
        painter.drawLine(x, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x, y + _data.cellHeight * 5 / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
    } else if (!hasLeft && hasRight && !hasTop && hasBottom) {
        // 右下
        painter.drawLine(x + _data.cellWidth * 5 / 6, y, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x, y + _data.cellHeight * 5 / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
    } else if (hasLeft && hasRight && hasTop && hasBottom) {
        // 左上下右
        drawFunctionName(painter, cell, x, y);
        painter.drawLine(x, y + _data.cellHeight / 2, x + _data.cellWidth / 6, y + _data.cellHeight / 2);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight / 6, x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth / 6, y + _data.cellHeight * 5 / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
        painter.drawLine(x + _data.cellWidth * 5 / 6, y + _data.cellHeight / 6, x + _data.cellWidth * 5 / 6, y + _data.cellHeight * 5 / 6);
    } else {
    }
}

void LadPaintWidget::drawFunctionName(QPainter& painter, const LadCell& cell, int x, int y) {
    QString opcodeStr;
    _data.opcodeToString(cell.opcode, opcodeStr);
    painter.drawText(QRectF(x + _data.cellWidth / 4, y + 3 * _data.cellHeight / 8, _data.cellWidth / 2, _data.cellHeight / 4), Qt::AlignCenter, opcodeStr);
}

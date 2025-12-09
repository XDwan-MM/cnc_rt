#pragma once
#include <QDialog>

namespace Ui {
class EventSimulatorDialog;
}

class EventSimulatorDialog : public QDialog {
    Q_OBJECT

public:
    explicit EventSimulatorDialog(QWidget* parent = nullptr);
    ~EventSimulatorDialog();

signals:
    void btnLdClicked();
    void btnLdiClicked();
    void btnOutClicked();
    void btnOoutClicked();
    void btnHLineClicked();
    void btnVLineClicked();
    void btnDeleteItemClicked();
    void btnDeleteVLineClicked();
    void btnAddRowClicked();
    void btnDeleteRowClicked();
    void inputFinished(const QString& text);

private:
    Ui::EventSimulatorDialog* ui;
};

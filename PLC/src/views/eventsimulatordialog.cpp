#include "eventsimulatordialog.h"
#include "ui_eventsimulatordialog.h"
#include <QDebug>

EventSimulatorDialog::EventSimulatorDialog(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::EventSimulatorDialog) {
    ui->setupUi(this);

    connect(ui->btnLd, &QPushButton::clicked, this, [this]() { emit btnLdClicked(); });
    connect(ui->btnLdi, &QPushButton::clicked, this, [this]() { emit btnLdiClicked(); });
    connect(ui->btnOut, &QPushButton::clicked, this, [this]() { emit btnOutClicked(); });
    connect(ui->btnOout, &QPushButton::clicked, this, [this]() { emit btnOoutClicked(); });
    connect(ui->btnHLine, &QPushButton::clicked, this, [this]() { emit btnHLineClicked(); });
    connect(ui->btnVLine, &QPushButton::clicked, this, [this]() { emit btnVLineClicked(); });
    connect(ui->btnDeleteItem, &QPushButton::clicked, this, [this]() { emit btnDeleteItemClicked(); });
    connect(ui->btnDeleteVline, &QPushButton::clicked, this, [this]() { emit btnDeleteVLineClicked(); });
    connect(ui->btnAddRow, &QPushButton::clicked, this, [this]() { emit btnAddRowClicked(); });
    connect(ui->btnDeleteRow, &QPushButton::clicked, this, [this]() { emit btnDeleteRowClicked(); });
    connect(ui->txtInput, &QLineEdit::editingFinished, this, [this]() {
        QString text = ui->txtInput->text();
        emit inputFinished(text);
        ui->txtInput->setText("");
    });
}

EventSimulatorDialog::~EventSimulatorDialog() {
    delete ui;
}

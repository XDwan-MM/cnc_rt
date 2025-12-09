#include "mainwindow.h"
#include "ladview.h"
#include "shmapi.h"
#include <QDebug>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {

    shmInit();
    resize(1024, 576);

    QWidget* centralwidget = new QWidget(this);
    QVBoxLayout* verticalLayout = new QVBoxLayout(centralwidget);
    verticalLayout->setContentsMargins(0, 0, 0, 0);
    verticalLayout->setSpacing(0);
    _ladView = new LadView(centralwidget);
    verticalLayout->addWidget(_ladView);

    setCentralWidget(centralwidget);
}

MainWindow::~MainWindow() {
    // shmDetach();
}

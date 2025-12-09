#pragma once

#include <QMainWindow>

class LadView;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    LadView* _ladView;
};

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "calibratedialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    CalibrateDialog *calDialog;

public:
    void showClibrateDialog();
    void connectHsRobotBnt();
    void enanleHsRobotBnt();
    void loadHSRobotPrgBnt();
    void HsRobotStartBnt();
    void HsrRobotStopBnt();
    void OpenOrCloseMicrophoneBnt();
    void OpenOrCloseVoiceRecognitionBnt();
    void StartPrizeClawBnt();
};

#endif // MAINWINDOW_H

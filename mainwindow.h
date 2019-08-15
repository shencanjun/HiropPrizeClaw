#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "calibratedialog.h"
#include "voicerecognite.h"

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
    VoiceRecognite *voice;
    ros::NodeHandle n_MW;

public:
    void sendNodeHanle(ros::NodeHandle n);
    void showClibrateDialog();
    void connectHsRobotBnt();
    void enanleHsRobotBnt();
    void loadHSRobotPrgBnt();
    void HsRobotStartBnt();
    void HsrRobotStopBnt();
    void OpenOrCloseVoiceRecognitionBnt();
    void StartPrizeClawBnt();
    void showVoiceRecognitionResult(QString str);
};

#endif // MAINWINDOW_H

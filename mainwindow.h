#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "calibratedialog.h"
#include "voicerecognite.h"
#include "hsc3robot.h"

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
    QTimer *getLocTimer;

public:
    HSC3ROBOT *hsc3;

public:
    void setReturnStrtoUI(QString str);

    void sendNodeHanle(ros::NodeHandle n);

    void initRobot();

    void showClibrateDialog();

    void connectHsRobotBnt();

    void enanleHsRobotBnt();

    void loadHSRobotPrgBnt();

    void HsRobotStartBnt();

    void OpenOrCloseVoiceRecognitionBnt();

    void StartPrizeClawBnt();

    void showVoiceRecognitionResult(QString str);

    void setResultHsrLR();

    void showHsrLocOnTime();
};

#endif // MAINWINDOW_H

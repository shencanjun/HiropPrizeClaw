#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QPainter>
#include <QTimer>
#include <sstream>
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
    QTimer *getHscMsgTimer;
    QTimer *showImageTimer;
    QPixmap imp;
    QString imageFileName;
    std::string progName;
    bool HscStatus;

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

    void HscMsgStatusLET();

    void HscClearFaultBnt();

    void showImagelabel();

    void showImageLabelChange();
};

#endif // MAINWINDOW_H

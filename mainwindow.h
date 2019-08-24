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
#include "camraoperate.h"

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
    QTimer *moveTimer;
    QPixmap imp;
    QString imageFileName;
    std::string camImageFileName;
    std::string camXmlFileName;
    std::string progName;
    bool HscStatus;
    int camX;
    int camY;
    bool isDetesion;
    int squareX;
    int squareY;
    int squareWidth;
    int squareHeigth;
    std::vector<double> detesionRPose;

public:
    HSC3ROBOT *hsc3;
    CamraOperate *camOpera;

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

    void connectCamraBnt();

    void camTakePirtureBnt();

    void camGetImageBnt();

    void detesionBnt();

    void getCamPose(geometry_msgs::Pose pose);

    void startMaulModeBnt();

    void startMove();

    void LabelDisplayMat(QLabel *label, cv::Mat &mat);
};

#endif // MAINWINDOW_H

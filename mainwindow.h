#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QPainter>
#include <QTimer>
#include <sstream>
#include <QEvent>
#include <QCloseEvent>
#include "calibratedialog.h"
#include "voicerecognite.h"
#include "hsc3robot.h"
#include "camraoperate.h"
#include "parseconfig.h"

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
    ParseConfig *parse;
    ros::NodeHandle n_MW;
    QTimer *getLocTimer;
    QTimer *getHscMsgTimer;
    QTimer *showgroupTimer;
    QTimer *moveTimer;
    QPixmap imp;
    QString MainXmlFile;
    QString imageFileName;
    QString calibXmlName;
    QString camImageFileName;
    QString camXmlFileName;

    std::string progName;
    std::string robotIpStr;
    uint16_t robotPort;
    int robotVord;

    bool HscStatus;
    int camX;
    int camY;
    bool isDetesion;
    int squareX;
    int squareY;
    int squareWidth;
    int squareHeigth;
    std::vector<double> detesionRPose;
    ObjType objType;

    double comX;
    double comY;

public:
    HSC3ROBOT *hsc3;
    CamraOperate *camOpera;

public:

    void readMainXml();

    void setReturnStrtoUI(QString str);

    void sendNodeHanle(ros::NodeHandle n);

    void readRobotConfig();

    void initRobot();

    void getRobotCom();

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

    void showImageLabelChange(cv::Mat draw);

    void connectCamraBnt();

    void camTakePirtureBnt();

    void camGetImageBnt();

    void detesionBnt();

    void getCamPose(geometry_msgs::PoseStamped pose);

    void startMaulModeBnt();

    void startMove();

    void selectObjCombobox(int index);

    void LabelDisplayMat(QLabel *label, cv::Mat &mat);

    void setFrameInCenter();

private:
    void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H

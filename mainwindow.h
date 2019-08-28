#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QPainter>
#include <QTimer>
#include <sstream>
#include <QEvent>
#include <QCloseEvent>
#include <boost/thread.hpp>
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
    QTimer *getHscMsgTimer;
    QTimer *showgroupTimer;
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
    int squareAX;
    int squareAY;
    int squareBX;
    int squareBY;
    int squareWidth;
    int squareHeigth;
    std::vector<double> detesionRPose;
    ObjType objType;
    bool isdone;

    double comX;
    double comY;

    boost::thread *thrd;
    boost::thread *moveThrd;
    boost::thread *HscLocThrd;

    bool hscThredflat;

public:
    HSC3ROBOT *hsc3;
    CamraOperate *camOpera;

public:
    void sendUIUpdataSignal(QString str) const{
        emit emitUIUpdata(str);
    }

    void sendDetectionUIUpdata(double rx,double ry) const{
        emit emitDetectionUIUpdata(rx, ry);
    }

    void sendHscLocData(LocData data) const{
        emit emitHscLocData(data);
    }

signals:
    void emitUIUpdata(QString str) const;
    void emitDetectionUIUpdata(double rx,double ry) const;
    void emitHscLocData(LocData) const;

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

    void getHscLocDataStart();

    void setResultHsrLR();

    void showHsrLocData(LocData);

    void getHscLocThrd();

    void HscMsgStatusLET();

    void HscClearFaultBnt();

    void showImagelabel();

    void showImageLabelChange(cv::Mat draw);

    void connectCamraBnt();

    void camTakePirtureBnt();

    void camGetImageBnt();

    void detesionBnt();

    void getCamPose(ObjType );

    void startMaulModeBnt();

    void startMove();

    void startMoveThrd();

    void selectObjCombobox(int index);

    void LabelDisplayMat(QLabel *label, cv::Mat &mat);

    void setFrameInCenter();

    void detectionThread();

    void detectionDone(bool,int, int, int);

    void showDetectionRobotData(double rx, double ry);

private:
    void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H

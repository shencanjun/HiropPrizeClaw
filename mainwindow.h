#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QPainter>
#include <QTimer>
#include <sstream>
#include <QEvent>
#include <QCloseEvent>
#include <QMouseEvent>
#include <QSound>
#include <QDebug>
#include <boost/thread.hpp>
#include "calibratedialog.h"
#include "ttsmscdialog.h"
#include "imagedialog.h"
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
    TtsMscDialog *ttsDialog;
    ImageDialog *imgDialog;
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
    QString camADeteImgFile;
    QString camSDeteImgFile;

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
    bool haveObj;
    bool movestop;

    double comX;
    double comY;

    int clikedcount;

    boost::thread *thrd;
    boost::thread *moveThrd;
    boost::thread *HscLocThrd;

    int voiceState;

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

    void sendCamPoseData(LocData pose) const{
        emit emitCamPoseData(pose);
    }

    void sendImg(cv::Mat mat) const{
        emit emitsendImgData(mat);
    }

signals:
    void emitUIUpdata(QString str) const;
    void emitDetectionUIUpdata(double rx,double ry) const;
    void emitHscLocData(LocData) const;
    void emitCamPoseData(LocData) const;
    void emitsendImgData(cv::Mat) const;

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

    void voiceStatus();

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

    bool getCamPose(ObjType );

    void startMaulModeBnt();

    void startMove();

    void startMoveThrd();

    void selectObjCombobox(int index);

    void LabelDisplayMat(QLabel *label, cv::Mat mat);

    void setFrameInCenter();

    void detectionThread();

    void detectionDone(bool,int, int, int, double);

    void showDetectionRobotData(double rx, double ry);

    bool getDetectionRobotPose();

    void showCamPose(LocData data);

    void updataImg();

    void showTtsMscDialog();

    void showImageDialog();

    void SendProgAction();

private:
    void closeEvent(QCloseEvent *event);
    bool eventFilter(QObject  *obj, QEvent *event);
};

#endif // MAINWINDOW_H

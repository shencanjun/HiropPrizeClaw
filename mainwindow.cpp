#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    voice = new VoiceRecognite(n_MW);
    hsc3 = new HSC3ROBOT();
    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
    connect(ui->pushButton_VoiceRecognition,&QPushButton::clicked,this,&MainWindow::OpenOrCloseVoiceRecognitionBnt);
    connect(ui->pushButton_connectRobot,&QPushButton::clicked,this,&MainWindow::connectHsRobotBnt);
    connect(ui->pushButton_ebnable,&QPushButton::clicked,this,&MainWindow::enanleHsRobotBnt);
    connect(ui->pushButton_Load,&QPushButton::clicked,this,&MainWindow::loadHSRobotPrgBnt);
    connect(ui->pushButton_start,&QPushButton::clicked,this,&MainWindow::HsRobotStartBnt);
    connect(voice,&VoiceRecognite::emitResultStr,this,&MainWindow::showVoiceRecognitionResult);
    connect(ui->pushButton_result_set,&QPushButton::clicked,this,&MainWindow::setResultHsrLR);
    initRobot();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::sendNodeHanle(ros::NodeHandle n)
{
    n_MW = n;
    return;
}

void MainWindow::setReturnStrtoUI(QString str)
{
    ui->VoiceRecognition_ReSponseTxt->append(str);
    ui->VoiceRecognition_ReSponseTxt->moveCursor(QTextCursor::End);
    return;
}

void MainWindow::showClibrateDialog()
{
    calDialog = new CalibrateDialog();
    calDialog->setWindowTitle("标定");
    calDialog->setHsc3Object(hsc3);
    calDialog->show();
}

void MainWindow::OpenOrCloseVoiceRecognitionBnt()
{
    if(ui->pushButton_VoiceRecognition->text() == "开始语音识别")
    {
        setReturnStrtoUI("开始语音识别......");
        ui->pushButton_VoiceRecognition->setText("停止语音识别");
        voice->startVoiceRecognition();
    }
    else
    {
        setReturnStrtoUI("停止语音识别......");
        ui->pushButton_VoiceRecognition->setText("开始语音识别");
        voice->stopVoiceRecognition();
    }
    return;
}

void MainWindow::showVoiceRecognitionResult(QString str)
{
    //QString pstr = ""+str;
    setReturnStrtoUI("抓取：<font color = blue size = 10>"+str+"</font>" );
    return;
}

void MainWindow::initRobot()
{
    bool en;
    if(hsc3->getHscEnanle(en))
    {
        if(en){
            ui->pushButton_ebnable->setText("失能");
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(85, 255, 127)");
        }
        else{
            ui->pushButton_ebnable->setText("使能");
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(255, 255, 255)");
        }
    }
}

void MainWindow::connectHsRobotBnt()
{
    std::string ipstr = ui->lineEdit_rip->text().toStdString();
    int8_t port = ui->lineEdit_rport->text().toInt();

    if(ui->pushButton_connectRobot->text() == "连接")
    {
        if(hsc3->connectIPC(ipstr,port))
         {
            ui->pushButton_connectRobot->setText("断开");
            ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(85, 255, 127)");
            ui->lineEdit_rip->setReadOnly(true);
            setReturnStrtoUI("连接机器人成功！！！");
         }
         else
         {
            ui->lineEdit_rip->setReadOnly(false);
            setReturnStrtoUI("<font color = red> 连接机器人失败！！！ </font>");
         }
    }
   else
    {
        if(hsc3->disconnectIPC())
        {
            ui->lineEdit_rip->setReadOnly(false);
           ui->pushButton_connectRobot->setText("连接");
           ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(２55, 255, 255)");
           setReturnStrtoUI("断开连接机器人成功！！！");
           ui->lineEdit_rip->setReadOnly(false);
        }
        else
        {
           ui->lineEdit_rip->setReadOnly(true);
           setReturnStrtoUI("<font color = red> 断开连接机器人失败！！！ </font>");
        }
    }
    return;
}

void MainWindow::enanleHsRobotBnt()
{
    if(ui->pushButton_ebnable->text() == "使能")
    {
        if(hsc3->setHscEnanle(true)){
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(85, 255, 127)");
            ui->pushButton_ebnable->setText("失能");
            setReturnStrtoUI("机器人使能成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人使能失败！！！ </font>");
        }
    }
    else
    {
        ui->pushButton_ebnable->setText("失能");
        if(hsc3->setHscEnanle(true)){
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(255, 255, 255)");
            ui->pushButton_ebnable->setText("使能");
            setReturnStrtoUI("机器人失能成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人使能失败！！！ </font>");
        }
    }
    return;
}

void MainWindow::loadHSRobotPrgBnt()
{
    std::string progname = "HIROP.PRG";
    if(ui->pushButton_Load->text() == "加载")
    {
        if(hsc3->HscLoadPRG(progname))
        {
            ui->pushButton_Load->setText("卸载");
            ui->pushButton_Load->setStyleSheet("background-color: rgb(85, 255, 127)");
            setReturnStrtoUI("机器人加载程序成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人加载程序失败！！！ </font>");
        }
    }
    else
    {
        if(hsc3->HscUnloadPRG(progname))
        {
            ui->pushButton_Load->setText("加载");
            ui->pushButton_Load->setStyleSheet("background-color: rgb(255, 255, 255)");
            setReturnStrtoUI("机器人卸载程序成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人卸载程序失败！！！ </font>");
        }
    }
    return;
}

void MainWindow::HsRobotStartBnt()
{
    if(ui->pushButton_start->text() == "开始" )
    {
        if(hsc3->HscPrgStart()){
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
            setReturnStrtoUI("开始运行机器人程序成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 开始运行机器人程序失败！！！ </font>");
        }
    }
    else
    {
        if(hsc3->HscPrgStop()){
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
            setReturnStrtoUI("停止运行机器人程序成功！！！");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 停止运行机器人程序失败！！！ </font>");
        }
    }
    return;
}

void MainWindow::setResultHsrLR()
{
    int index = 100;

    double lrX = ui->lineEdit_result_X->text().toDouble();
    double lrY = ui->lineEdit_result_Y->text().toDouble();
    double lrZ = ui->lineEdit_result_Z->text().toDouble();
    double lrA = ui->lineEdit_result_A->text().toDouble();
    double lrB = ui->lineEdit_result_B->text().toDouble();
    double lrC = ui->lineEdit_result_C->text().toDouble();

    LocData locdata;
    locdata.push_back(lrX);
    locdata.push_back(lrY);
    locdata.push_back(lrZ);
    locdata.push_back(lrA);
    locdata.push_back(lrB);
    locdata.push_back(lrC);

    if(hsc3->setHscLR(index, locdata))
    {
        setReturnStrtoUI("<font color = green> 设置机器人LR成功！！！ </font>");
    }
    else
    {
        setReturnStrtoUI("<font color = red> 设置机器人LR失败！！！ </font>");
    }

    return;

}


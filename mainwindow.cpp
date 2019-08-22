#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    progName = "HIROP.PRG";
    HscStatus = true;
    calDialog = new CalibrateDialog();
    calDialog->setWindowTitle("标定");
    voice = new VoiceRecognite(n_MW);
    hsc3 = new HSC3ROBOT();
    getLocTimer = new QTimer(this);
    getHscMsgTimer = new QTimer(this);
    showImageTimer = new QTimer(this);
    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
    connect(ui->pushButton_VoiceRecognition,&QPushButton::clicked,this,&MainWindow::OpenOrCloseVoiceRecognitionBnt);
    connect(ui->pushButton_connectRobot,&QPushButton::clicked,this,&MainWindow::connectHsRobotBnt);
    connect(ui->pushButton_ebnable,&QPushButton::clicked,this,&MainWindow::enanleHsRobotBnt);
    connect(ui->pushButton_Load,&QPushButton::clicked,this,&MainWindow::loadHSRobotPrgBnt);
    connect(ui->pushButton_start,&QPushButton::clicked,this,&MainWindow::HsRobotStartBnt);
    connect(voice,&VoiceRecognite::emitResultStr,this,&MainWindow::showVoiceRecognitionResult);
    connect(ui->pushButton_result_set,&QPushButton::clicked,this,&MainWindow::setResultHsrLR);
    connect(ui->pushButton_clearFault, &QPushButton::clicked, this, &MainWindow::HscClearFaultBnt);
    connect(getLocTimer, &QTimer::timeout, this, &MainWindow::showHsrLocOnTime);
    connect(getHscMsgTimer, &QTimer::timeout, this, &MainWindow::HscMsgStatusLET);
    connect(ui->actionOpenImage, &QAction::triggered, this, &MainWindow::showImagelabel);
    connect(showImageTimer, &QTimer::timeout, this, &MainWindow::showImageLabelChange);
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
    calDialog->setHsc3Object(hsc3);
    calDialog->show();
}

void MainWindow::OpenOrCloseVoiceRecognitionBnt()
{
    if(ui->pushButton_VoiceRecognition->text() == "开始语音识别")
    {
        setReturnStrtoUI("<font color = green size = 7> 开始语音识别...... </font>");
        ui->pushButton_VoiceRecognition->setText("停止语音识别");
        voice->startVoiceRecognition();
    }
    else
    {
        setReturnStrtoUI("<font color = red size = 7> 停止语音识别...... </font>");
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
    if(!HscStatus)
    {
        ui->lineEdit_Msg->setStyleSheet("background-color: rgb(255, 85, 0);");
    }
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
    const std::string fileName;
    ProgInfo info;
    if(hsc3->getHscProInfo(fileName, info))
    {
        if(info.state != AUTO_STATE_UNLOAD)
        {
            ui->pushButton_Load->setText("卸载");
            ui->pushButton_Load->setStyleSheet("background-color: rgb(85, 255, 127)");
        }
        else if(info.state == AUTO_STATE_RUNNING)
        {
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
        }
    }
}

void MainWindow::connectHsRobotBnt()
{
    std::string ipstr = ui->lineEdit_rip->text().toStdString();
    uint16_t port = ui->lineEdit_rport->text().toUShort();

    if(ui->pushButton_connectRobot->text() == "连接")
    {
        if(hsc3->connectIPC(ipstr,port))
         {
            ui->pushButton_connectRobot->setText("断开");
            ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(85, 255, 127)");
            ui->lineEdit_rip->setReadOnly(true);
            ui->lineEdit_rport->setReadOnly(true);
            getLocTimer->start(1.0);
            getHscMsgTimer->start(1.0);
            sleep(1);
            initRobot();
            setReturnStrtoUI("<font color = green> 连接机器人成功！！！</font>");
         }
         else
         {
            ui->lineEdit_rip->setReadOnly(false);
            ui->lineEdit_rport->setReadOnly(false);
            setReturnStrtoUI("<font color = red> 连接机器人失败！！！ </font>");
         }
    }
   else
    {
        if(hsc3->disconnectIPC())
        {
            ui->lineEdit_rip->setReadOnly(false);
            ui->lineEdit_rport->setReadOnly(false);
            ui->pushButton_connectRobot->setText("连接");
            ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(２55, 255, 255)");
            getLocTimer->stop();
            getHscMsgTimer->stop();
            setReturnStrtoUI("<font color = green> 断开连接机器人成功！！！</font>");
        }
        else
        {
           ui->lineEdit_rip->setReadOnly(true);
           ui->lineEdit_rport->setReadOnly(true);
           setReturnStrtoUI("<font color = red> 断开连接机器人失败！！！ </font>");
        }
    }
    return;
}

void MainWindow::enanleHsRobotBnt()
{
    if(ui->pushButton_ebnable->text() == "使能")
    {
        if(hsc3->setHscEnanle(true) && HscStatus){
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(85, 255, 127)");
            ui->pushButton_ebnable->setText("失能");
            setReturnStrtoUI("<font color = green> 机器人使能成功！！！</font>");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人使能失败！！！ </font>");
        }
    }
    else
    {
        ui->pushButton_ebnable->setText("失能");
        if(hsc3->setHscEnanle(false)){
            ui->pushButton_ebnable->setStyleSheet("background-color: rgb(255, 255, 255)");
            ui->pushButton_ebnable->setText("使能");
            setReturnStrtoUI("<font color = green> 机器人失能成功！！！<font>");
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
    if(ui->pushButton_Load->text() == "加载")
    {
        if(hsc3->HscLoadPRG(progName) && HscStatus)
        {
            ui->pushButton_Load->setText("卸载");
            ui->pushButton_Load->setStyleSheet("background-color: rgb(85, 255, 127)");
            setReturnStrtoUI("<font color = green> 机器人加载程序成功！！！</font>");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 机器人加载程序失败！！！ </font>");
        }
    }
    else
    {
        if(hsc3->HscUnloadPRG(progName))
        {
            ui->pushButton_Load->setText("加载");
            ui->pushButton_Load->setStyleSheet("background-color: rgb(255, 255, 255)");
            setReturnStrtoUI("<font color = green> 机器人卸载程序成功！！！</font>");
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
        if(hsc3->HscPrgStart(progName) && HscStatus){
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
            setReturnStrtoUI("<font color = green> 开始运行机器人程序成功！！！</font>");
        }
        else
        {
            setReturnStrtoUI("<font color = red> 开始运行机器人程序失败！！！ </font>");
        }
    }
    else
    {
        if(hsc3->HscPrgStop(progName)){
            ui->pushButton_start->setText("开始");
            ui->pushButton_start->setStyleSheet("background-color: rgb(255, 255, 255)");
            setReturnStrtoUI("<font color = green> 停止运行机器人程序成功！！！</font>");
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

    double lrArry[9] = {lrX, lrY, lrZ, lrA, lrB, lrC, 0, 0, 0};

    LocData locdata;
    for(int i = 0; i < 9; i++)
    {
        locdata.push_back(lrArry[i]);
    }

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

void MainWindow::showHsrLocOnTime()
{
    LocData locdata;
    int bitNum = 2;

    if(!hsc3->getHscLoc(locdata))
    {
        setReturnStrtoUI("<font color = red> 获取机器人笛卡尔坐标失败！！！ </font>");
        return;
    }
    if(locdata.size() < 6)
        return;

    ui->label_X->setText(QString::number(locdata[0], 'f', bitNum));
    ui->label_Y->setText(QString::number(locdata[1], 'f', bitNum));
    ui->label_Z->setText(QString::number(locdata[2], 'f', bitNum));
    ui->label_A->setText(QString::number(locdata[3], 'f', bitNum));
    ui->label_B->setText(QString::number(locdata[4], 'f', bitNum));
    ui->label_C->setText(QString::number(locdata[5], 'f', bitNum));

    return;

}

void MainWindow::HscMsgStatusLET()
{
    ErrLevel level;
    std::string msg;
    QString qstr;

    if(!hsc3->getFaultMessage(level,msg))
    {
        setReturnStrtoUI("<font color = red> 获取机器人报警！！！ </font>");
        return;
    }
    qstr = QString::fromStdString(msg);
    if(level == ERR_LEVEL_ERROR || level == ERR_LEVEL_FATAL)
    {
        HscStatus = false;
        ui->lineEdit_Msg->setStyleSheet("background-color: rgb(255, 85, 0);");

        setReturnStrtoUI("<font color = red>" + qstr + "</font>");
    }
    else if(level == ERR_LEVEL_WARNING)
    {
        setReturnStrtoUI("<font color = yellow>" + qstr + "</font>");
    }
    else
    {
        setReturnStrtoUI("<font color = black>" + qstr + "</font>");
    }

    return;
}

void MainWindow::HscClearFaultBnt()
{
    if(!hsc3->HscClearFault())
    {
        setReturnStrtoUI("<font color = red> 机器人复位失败！！！ </font>");
        return;
    }
    setReturnStrtoUI("<font color = red> 机器人复位成功！！！ </font>");
    return;
}

void MainWindow::showImagelabel()
{
    //QImage *img = new QImage();
    showImageTimer->stop();
    imageFileName = QFileDialog::getOpenFileName(this,"打开图片","","Image (*.png *.bmp *.jpg *.tif *.GIF )");
    if(imageFileName.isEmpty())
        return;
    //imp.load(imageFileName);
    if(!(imp.load(imageFileName)))
    {
        {
             QMessageBox::information(this,
                                             tr("打开图像失败"),
                                             tr("打开图像失败!"));
             delete &imp;
             return;
         }
    }


    showImageTimer->start(100);

//    QPixmap nimp = imp.scaled(ui->label_show_image->width(),ui->label_show_image->height());

//    QPainter painter(this);

//    painter.drawPixmap(0,0,nimp);

//    ui->label_show_image->setPixmap(nimp);
    return;
}

void MainWindow::showImageLabelChange()
{
    if(&imp == NULL)
        return;
    QPixmap nimp = imp.scaled(ui->label_show_image->width(),ui->label_show_image->height());
    QPainter painter(this);

    painter.drawPixmap(0,0,nimp);

    ui->label_show_image->setPixmap(nimp);
}

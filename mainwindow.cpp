#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");

    progName = "HIROP.PRG";
    camImageFileName ="/home/fshs/show.png";
    camXmlFileName = "./calibrate.xml";
    objType = DOG;
    isDetesion = false;
    HscStatus = true;
    calDialog = new CalibrateDialog();
    calDialog->setWindowTitle("标定");
    voice = new VoiceRecognite(n_MW);
    hsc3 = new HSC3ROBOT();

    camOpera = new CamraOperate(n_MW);
    camOpera->imgFileName = camImageFileName;
    camOpera->camCalibXmlFileName = camXmlFileName;

    getLocTimer = new QTimer(this);
    getHscMsgTimer = new QTimer(this);
    showImageTimer = new QTimer(this);
    moveTimer = new QTimer(this);

    //连接信号槽
    connect(voice,&VoiceRecognite::emitResultStr,this,&MainWindow::showVoiceRecognitionResult);
    connect(camOpera, &CamraOperate::emitResultCam, this, &MainWindow::getCamPose);
    connect(camOpera, &CamraOperate::emitImagesignal, this, &MainWindow::showImagelabel);

    connect(getLocTimer, &QTimer::timeout, this, &MainWindow::showHsrLocOnTime);
    connect(showImageTimer, &QTimer::timeout, this, &MainWindow::setFrameInCenter);
    connect(moveTimer, &QTimer::timeout, this, &MainWindow::startMove);
    connect(getHscMsgTimer, &QTimer::timeout, this, &MainWindow::HscMsgStatusLET);

    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
    connect(ui->pushButton_VoiceRecognition,&QPushButton::clicked,this,&MainWindow::OpenOrCloseVoiceRecognitionBnt);
    connect(ui->pushButton_connectRobot,&QPushButton::clicked,this,&MainWindow::connectHsRobotBnt);
    connect(ui->pushButton_ebnable,&QPushButton::clicked,this,&MainWindow::enanleHsRobotBnt);
    connect(ui->pushButton_Load,&QPushButton::clicked,this,&MainWindow::loadHSRobotPrgBnt);
    connect(ui->pushButton_start,&QPushButton::clicked,this,&MainWindow::HsRobotStartBnt);
    connect(ui->pushButton_result_set,&QPushButton::clicked,this,&MainWindow::setResultHsrLR);
    connect(ui->pushButton_clearFault, &QPushButton::clicked, this, &MainWindow::HscClearFaultBnt);
    connect(ui->actionOpenImage, &QAction::triggered, this, &MainWindow::showImagelabel);
    connect(ui->pushButton_openCam, &QPushButton::clicked, this, &MainWindow::connectCamraBnt);
    connect(ui->pushButton_takePirture, &QPushButton::clicked, this, &MainWindow::camTakePirtureBnt);
    connect(ui->pushButton_detesion, &QPushButton::clicked, this, &MainWindow::detesionBnt);
    connect(ui->pushButton_PrizeClaw, &QPushButton::clicked, this, &MainWindow::startMaulModeBnt);
    connect(ui->comboBox_object, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainWindow::selectObjCombobox);
    //connect(ui->comboBox_object,SIGNAL(currentIndexChanged(int)),this,SLOT(selectObjCombobox(int)));

    showImageTimer->start(300);
    camOpera->startCamraService();
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
    calDialog->setOpeaObject(hsc3, camOpera);
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
            getLocTimer->start(100);
            getHscMsgTimer->start(100);
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

    std::string path = imageFileName.toStdString();

    camOpera->colorImg = cv::imread(path);

    //showImageTimer->start();

    //ui->label_show_image->clear();

    //LabelDisplayMat(ui->label_show_image,camOpera->colorImg);

    showImageLabelChange();

    return;
}

void MainWindow::showImageLabelChange()
{
    ui->label_show_image->clear();

    LabelDisplayMat(ui->label_show_image,camOpera->colorImg);
}

void MainWindow::connectCamraBnt()
{
    if(ui->pushButton_openCam->text() == "打开相机"){
        if(camOpera->connectCamra() == 0){
            ui->pushButton_openCam->setText("关闭相机");
            setReturnStrtoUI("<font color = green> 打开相机成功!!! </font>");
        }
        else if(camOpera->connectCamra() == -2){
            ui->pushButton_openCam->setText("关闭相机");
            setReturnStrtoUI("<font color = green> 相机已经打开!!! </font>");
        }
        else {
            setReturnStrtoUI("<font color = red> 打开相机失败!!! </font>");
        }
    }
    else{
        if(camOpera->disconnectCamra()){
            ui->pushButton_openCam->setText("打开相机");
            setReturnStrtoUI("<font color = green> 关闭相机成功!!! </font>");
        }
        else {
            setReturnStrtoUI("<font color = red> 关闭相机失败!!! </font>");
        }
    }
}

void MainWindow::camTakePirtureBnt()
{
    if(!camOpera->takePicture()){
        setReturnStrtoUI("<font color = red> 相机拍照失败!!! </font>");
        return;
    }
    setReturnStrtoUI("<font color = green> 相机拍照成功!!! </font>");

    if(!camOpera->getImage()){
        setReturnStrtoUI("<font color = red> 获取图像!!! </font>");
        return;
    }
    setReturnStrtoUI("<font color = green> 获取图像!!! </font>");
    return;
}

void MainWindow::camGetImageBnt()
{
    if(!camOpera->getImage()){
        setReturnStrtoUI("<font color = red> 获取图像!!! </font>");
        return;
    }
    setReturnStrtoUI("<font color = green> 获取图像!!! </font>");
    return;
}

void MainWindow::detesionBnt()
{
    if(!camOpera->takePicture())
        return;

    if(!camOpera->getImage())
        return;

    if(!camOpera->detectionSrv())
        return;

    sleep(1.0);

    if(!isDetesion){
        std::cout<< "识别失败："<<isDetesion<<std::endl;
        setReturnStrtoUI("<font color = red> 识别娃娃失败!!! </font>");
        return;
    }

    if(!camOpera->getDetesionResult(camX,camY,detesionRPose))
        return;

    ui->lineEdit_result_X->setText(QString::number(detesionRPose[0],'f',4));
    ui->lineEdit_result_Y->setText(QString::number(detesionRPose[1],'f',4));
    ui->lineEdit_result_Z->setText(QString::number(detesionRPose[2],'f',4));
    ui->lineEdit_result_A->setText(QString::number(detesionRPose[3],'f',4));
    ui->lineEdit_result_B->setText(QString::number(detesionRPose[4],'f',4));
    ui->lineEdit_result_C->setText(QString::number(detesionRPose[5],'f',4));

    LocData locData;
    locData = detesionRPose;
    if(!hsc3->setHscLR(100, locData))
    {
        setReturnStrtoUI("<font color = red> 识别娃娃失败,LR!!! </font>");
        return;
    }

    return;
}

void MainWindow::getCamPose(geometry_msgs::Pose pose)
{
    setReturnStrtoUI("<font color = green> getCamPose!!! </font>");
    std::cout<<"32132131231231231"<<std::endl;
    //１，识别成功　０，识别失败
    isDetesion = static_cast<bool>(pose.position.z);

    //识别的相机坐标
    camX = static_cast<int>(pose.position.x);
    camY = static_cast<int>(pose.position.y);

    //画框尺寸
    squareX = static_cast<int>(pose.orientation.w);
    squareY = static_cast<int>(pose.orientation.x);
    squareWidth = static_cast<int>(pose.orientation.y);
    squareHeigth = static_cast<int>(pose.orientation.z);

    if(isDetesion)
        if(!camOpera->opencvDrawing(squareX,squareY,squareWidth,squareHeigth))
            return;

    return;
}

void MainWindow::startMove()
{
    double value = 0;
    int attemp = 0;
    hsc3->setHscR(10,1);
    do{
        hsc3->getHscR(11,value);
        std::cout<<"start_move"<<std::endl;
        sleep(1.0);
        ++attemp;
    }
    while(value == 0.0 && attemp < 3);

    if(attemp >= 3)
    {
        setReturnStrtoUI("<font color = green> 机器人到位失败!!! </font>");
        moveTimer->stop();
        return;
    }

    setReturnStrtoUI("<font color = green> 机器人已到位!!! </font>");

    //识别
    detesionBnt();

    setReturnStrtoUI("<font color = green> 识别成功，正在抓取!!! </font>");

    if(!hsc3->setHscR(12,1))
    {
        setReturnStrtoUI("<font color = red> 抓取成功!!! </font>");
        moveTimer->stop();
        return;
    }

    setReturnStrtoUI("<font color = green> 抓取成功!!! </font>");
    moveTimer->stop();
    return;
}

void MainWindow::startMaulModeBnt()
{
    moveTimer->start(100);
    return;
}

void MainWindow::selectObjCombobox(int index)
{
    switch (index) {
    case 0:
        objType = DOG;
        setReturnStrtoUI("<font color = green > 选择小狗狗 </font>");
        break;
    case 1:
        objType = RABBIT;
        setReturnStrtoUI("<font color = green > 选择小兔兔 </font>");
        break;
    case 2:
        objType = MONKEY;
        setReturnStrtoUI("<font color = green > 选择小猴子 </font>");
        break;
    default:
        objType = NONE;
        break;
    }
}

void MainWindow::LabelDisplayMat(QLabel *label, cv::Mat &mat)
{
    cv::Mat Rgb;
    QImage Img;
    if (mat.channels() == 3)//RGB Img
    {
        cv::cvtColor(mat, Rgb, CV_BGR2RGB);//颜色空间转换
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGB888);
    }
    else//Gray Img
    {
        Img = QImage((const uchar*)(mat.data), mat.cols, mat.rows, mat.cols*mat.channels(), QImage::Format_Indexed8);
    }

    label->setScaledContents(true) ;
    QSize qs = label->rect().size();

    QPixmap qimp = QPixmap::fromImage(Img);
    QPixmap nimp = qimp.scaled(qs);

    label->setPixmap(nimp);
}

void MainWindow::setFrameInCenter()
{
    int h = 30;

    int gw = ui->groupBox_vision->geometry().width();
    int gh = ui->groupBox_vision->geometry().height();

    int fw = ui->frame_vision->geometry().width();
    int fh = ui->frame_vision->geometry().height();

    int fx = (gw - fw) / 2;
    int fy = (gh - fh) / 2;

    fy = h;
    fh = gh - (h*2) + 10;

    std::cout<<"gw:" <<gw<<std::endl
              <<"gh:" <<gh<<std::endl
              <<"fw:" <<fw <<std::endl
              <<"fh:" <<fh <<std::endl
              <<"fx:" <<fx <<std::endl
              <<"fy:" <<fy <<std::endl;

    ui->frame_vision->setGeometry(fx,fy,fw,fh);

    return;
}

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //qRegisterMetaType<geometry_msgs::PoseStamped>("geometry_msgs::PoseStamped");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<LocData>("LocData");

   // MainXmlFile = "/home/fshs/HiropPrizeClaw/build-HiropPrizeClaw-unknown-Debug/MainConfig.xml";

    progName = "HIROP.PRG";
    camImageFileName ="/home/fshs/PrizeClaw/show.jpg";
    camADeteImgFile = "/home/fshs/prizeClaw/Adetetion.jpg";
    camSDeteImgFile = "/home/fshs/prizeClaw/Sdetetion.jpg";
    camXmlFileName = "calibrate";
    calibXmlName = "/home/fshs/prizeClaw/calibrateData.xml";

    objType = BEAR;
    voiceState = 0;
    clikedcount = 0;

    isDetesion = false;
    isdone = false;
    haveObj = true;
    HscStatus = true;
    movestop = true;
    calDialog = new CalibrateDialog();
    calDialog->setWindowTitle("标定");
    calDialog->calibDataFileName = calibXmlName;
    voice = new VoiceRecognite(n_MW);
    hsc3 = new HSC3ROBOT();

    ttsDialog = new TtsMscDialog();
    imgDialog = new ImageDialog();
    imgDialog->initCamObj(camOpera);

    camOpera = new CamraOperate(n_MW);
    camOpera->imgFileName = camImageFileName.toStdString();
    camOpera->camCalibXmlFileName = camXmlFileName.toStdString();
    camOpera->AdetecImgFIle = camADeteImgFile;

    parse = new ParseConfig();

   // readMainXml();
    getHscMsgTimer = new QTimer(this);
    showgroupTimer = new QTimer(this);

    ui->label_show_image->installEventFilter(this);
    //信号槽
    connect(voice, &VoiceRecognite::emitResultStr,this,&MainWindow::showVoiceRecognitionResult);
    connect(camOpera, &CamraOperate::emitResultCam, this, &MainWindow::detectionDone);
    connect(camOpera, &CamraOperate::emitImagesignal, this, &MainWindow::showImageLabelChange);
    connect(calDialog, &CalibrateDialog::emitComSignal, this, &MainWindow::getRobotCom);
    connect(this, &MainWindow::emitUIUpdata, this, &MainWindow::setReturnStrtoUI);
    connect(this, &MainWindow::emitDetectionUIUpdata, this, &MainWindow::showDetectionRobotData);
    connect(this, &MainWindow::emitHscLocData, this, &MainWindow::showHsrLocData);
    connect(this, &MainWindow::emitCamPoseData, this, &MainWindow::showCamPose);
    connect(this, &MainWindow::emitsendImgData, imgDialog, &ImageDialog::reviceImg);

    connect(showgroupTimer, &QTimer::timeout, this, &MainWindow::setFrameInCenter);
    //connect(getHscMsgTimer, &QTimer::timeout, this, &MainWindow::HscMsgStatusLET);

    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
    connect(ui->actionVoiceTts, &QAction::triggered,this,&MainWindow::showTtsMscDialog);
    connect(ui->actiontImage,&QAction::triggered, this, &MainWindow::showImageDialog);
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

    showgroupTimer->start(300);
    camOpera->startCamraService();
    getRobotCom();
    //readRobotConfig();
}

MainWindow::~MainWindow()
{
    delete parse;
    delete voice;
    delete calDialog;
    delete ui;
}

void MainWindow::sendNodeHanle(ros::NodeHandle n)
{
    n_MW = n;
    return;
}

void MainWindow::getRobotCom()
{
    double acc;
    if(parse->readCalibXML(calibXmlName,acc,comX,comY)){
        setReturnStrtoUI("<font color = green> 获取补偿值成功 </font>");
        return;
    }
    //setReturnStrtoUI("<font color = red> 获取补偿值失败 </font>");
    return;
}

void MainWindow::readMainXml()
{
    QString configStr;
    if(parse->readMainXml(MainXmlFile,"urldata","camImageFileName",configStr))
        return;
    std::cout<<"configStr:"<<configStr.toStdString()<<std::endl;
    camImageFileName = configStr;
    if(parse->readMainXml(MainXmlFile,"urldata","camXmlFileName",configStr))
        return;
    std::cout<<"configStr:"<<configStr.toStdString()<<std::endl;
    camXmlFileName = configStr;
    if(parse->readMainXml(MainXmlFile,"urldata","calibXmlName",configStr))
        return;
    std::cout<<"configStr:"<<configStr.toStdString()<<std::endl;
    calibXmlName = configStr;
    return;
}

void MainWindow::setReturnStrtoUI(QString str)
{
    QStringList strlist = str.split("<");
    if(strlist.size() >= 1){
        strlist = strlist[1].split(">");
        if(strlist.size() >= 1){
            for(int i=0;i<strlist.size();i++)
            {
                qDebug()<<strlist[i];
            }
            voice->textToSoundPlay(strlist[1]);
        }
    }
    ui->VoiceRecognition_ReSponseTxt->append(str);
    ui->VoiceRecognition_ReSponseTxt->moveCursor(QTextCursor::End);
    return;
}

void MainWindow::showClibrateDialog()
{
    calDialog->setOpeaObject(hsc3, camOpera);
    calDialog->readCalibrateData();
    calDialog->show();
}

void MainWindow::showTtsMscDialog()
{
    ttsDialog->setWindowTitle("语音合成");
    ttsDialog->show();
}

void MainWindow::OpenOrCloseVoiceRecognitionBnt()
{
    if(ui->pushButton_VoiceRecognition->text() == "开始语音识别")
    {
        setReturnStrtoUI("<font color = green size = 7> 开始语音识别...... </font>");
        ui->pushButton_VoiceRecognition->setText("停止语音识别");
        voice->startVoiceRecognition();
        //voice->textToSoundPlay("开始语音识别");
        voiceState = 1;
    }
    else
    {
        setReturnStrtoUI("<font color = red size = 7> 停止语音识别...... </font>");
        ui->pushButton_VoiceRecognition->setText("开始语音识别");
        voice->stopVoiceRecognition();
        //voice->textToSoundPlay("停止语音识别");
        voiceState = 2;
    }
    return;
}

void MainWindow::showVoiceRecognitionResult(QString str)
{
    //QString pstr = ""+str;
    setReturnStrtoUI("抓取：<font color = blue size = 10>"+str+"</font>" );
    if(str== "熊"){
        selectObjCombobox(0);
    }
    else if(str == "兔"){
        selectObjCombobox(1);
    }
    else if(str == "长颈鹿"){
        selectObjCombobox(2);
    }
    else{
        return;
    }
    voice->stopVoiceRecognition();
    voiceState = 2;
    startMove();
    return;
}

void MainWindow::readRobotConfig()
{
    QString qrobotIPStr,qrobotPortStr,qrobotPrgNameStr,qrobotVordStr,qrobotIsStr;
    if(parse->readMainXml(MainXmlFile,"robot","default_ip",qrobotIPStr))
        robotIpStr = qrobotIPStr.toStdString();
    if(parse->readMainXml(MainXmlFile,"robot","default_port",qrobotPortStr))
        robotPort = qrobotPortStr.toUShort();
    if(parse->readMainXml(MainXmlFile,"robot","progname",qrobotPrgNameStr))
        progName = qrobotPrgNameStr.toStdString();
    if(parse->readMainXml(MainXmlFile,"robot","speed",qrobotVordStr))
        robotVord = qrobotVordStr.toInt();
    if(parse->readMainXml(MainXmlFile,"robot","isconnect",qrobotIsStr)){
        if(qrobotIsStr == "true" || qrobotIsStr == "True" || qrobotIsStr == "TRUE"){
            if(hsc3->connectIPC(robotIpStr,robotPort))
            {
                ui->pushButton_connectRobot->setText("断开");
                ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(85, 255, 127)");
                ui->lineEdit_rip->setReadOnly(true);
                ui->lineEdit_rport->setReadOnly(true);
                sleep(1);
                initRobot();
                hscThredflat = true;
                getHscLocDataStart();
                setReturnStrtoUI("<font color = green> 连接机器人成功！！！</font>");
            }
            else
            {
                ui->lineEdit_rip->setReadOnly(false);
                ui->lineEdit_rport->setReadOnly(false);
                setReturnStrtoUI("<font color = red> 连接机器人失败！！！ </font>");
            }
        }
    }
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
            std::cout<<"sadsder"<<std::endl;
            ui->pushButton_connectRobot->setText("断开");
            ui->pushButton_connectRobot->setStyleSheet("background-color: rgb(85, 255, 127)");
            ui->lineEdit_rip->setReadOnly(true);
            ui->lineEdit_rport->setReadOnly(true);
            getHscMsgTimer->start(100);
            sleep(1);
            initRobot();
            hscThredflat = true;
            getHscLocDataStart();
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
            getHscMsgTimer->stop();
            hscThredflat = false;
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
        hsc3->setHscMode(OP_AUT);
        hsc3->setHscVord(20);
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

void MainWindow::getHscLocDataStart()
{
    boost::function0<void > f = boost::bind(&MainWindow::getHscLocThrd, this);
    HscLocThrd = new boost::thread(f);
}

void MainWindow::showHsrLocData(LocData data)
{
    int bitNum = 2;

    if(data.size() < 6)
        return;

    ui->label_X->setText(QString::number(data[0], 'f', bitNum));
    ui->label_Y->setText(QString::number(data[1], 'f', bitNum));
    ui->label_Z->setText(QString::number(data[2], 'f', bitNum));
    ui->label_A->setText(QString::number(data[3], 'f', bitNum));
    ui->label_B->setText(QString::number(data[4], 'f', bitNum));
    ui->label_C->setText(QString::number(data[5], 'f', bitNum));

    return;

}

void MainWindow::getHscLocThrd()
{
    while(hscThredflat){
        LocData locdata;
        if(!hsc3->getHscLoc(locdata))
        {
            emitUIUpdata("<font color = red> 获取机器人笛卡尔坐标失败！！！ </font>");
            return;
        }

        sendHscLocData(locdata);
        sleep(1);
    }
    return;
}

void MainWindow::HscMsgStatusLET()
{
    ErrLevel level;
    std::string msg;
    QString qstr;

    if(!hsc3->getFaultMessage(level,msg))
    {
        //setReturnStrtoUI("<font color = red> 获取机器人报警！！！ </font>");
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
    imageFileName = QFileDialog::getOpenFileName(this,"打开图片","","Image (*.png *.bmp *.jpg *.tif *.GIF )");
    if(imageFileName.isEmpty())
        return;

    std::string path = imageFileName.toStdString();

    camOpera->colorImg = cv::imread(path);

    //showImageLabelChange(camOpera->colorImg);
    camOpera->sendImage(camOpera->colorImg);
    sendImg(camOpera->colorImg);

    return;
}

void MainWindow::showImageLabelChange(cv::Mat draw)
{
    ui->label_show_image->clear();

    LabelDisplayMat(ui->label_show_image,draw);

    return;
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
        emitUIUpdata("<font color = red> 相机拍照失败!!! </font>");
        return;
    }
    emitUIUpdata("<font color = green> 相机拍照成功!!! </font>");

    if(!camOpera->getImage()){
        emitUIUpdata("<font color = red> 获取图像!!! </font>");
        return;
    }
    emitUIUpdata("<font color = green> 获取图像!!! </font>");
    return;
}

void MainWindow::camGetImageBnt()
{
    if(!camOpera->getImage()){
        emitUIUpdata("<font color = red> 获取图像!!! </font>");
        return;
    }
    emitUIUpdata("<font color = green> 获取图像!!! </font>");
    return;
}

void MainWindow::detesionBnt()
{
    boost::function0<void > f = boost::bind(&MainWindow::detectionThread, this);
    thrd = new boost::thread(f);
}

void MainWindow::detectionThread()
{
   int attemp = 0;
   isdone = false;
   haveObj = false;
   if(!camOpera->takePicture()){
        haveObj = false;
        return;
   }

    if(!camOpera->getImage()){
        haveObj = false;
        return;
    }

    usleep(100000);

    if(!camOpera->detectionSrv()){
        haveObj = false;
        return;
    }

    while(!isdone && attemp <= 10){
        usleep(300000);
        attemp++;
    }
    if(attemp >= 10){
        emitUIUpdata("<font color = red> 识别等待超时!!! </font>");
        haveObj = false;
        return;
    }

    isdone = false;

    if(!getDetectionRobotPose()){
        haveObj = false;
        return;
    }

    haveObj = true;
    return;
}

bool MainWindow::getCamPose(ObjType type)
{
    //vector<geometry_msgs::Pose>::iterator k;
    geometry_msgs::Pose pose;
    LocData data;

    if(type == BEAR){
        if(camOpera->vecBear.size() <= 0){
            emitUIUpdata("<font color = red> 没有小熊了!!! </font>");
            return false;
        }
        pose = camOpera->vecBear[0];
        std::cout<<"BEAR"<<std::endl;
    }
    else if(type == RABBIT){
        if(camOpera->vecRabbit.size() <= 0){
            emitUIUpdata("<font color = red> 没有小兔子了!!! </font>");
            return false;
        }
        pose = camOpera->vecRabbit[0];
        std::cout<<"RABBIT"<<std::endl;
    }
    else if(type == GIRAFFE){
        if(camOpera->vecGiraffe.size() <= 0){
            emitUIUpdata("<font color = red> 没有长颈鹿了!!! </font>");
            return false;
        }
        pose = camOpera->vecGiraffe[0];
        std::cout<<"GIRAFFE"<<std::endl;
    }
    else {
        return false;
    }

    //画框尺寸
    squareAX = static_cast<int>(pose.orientation.x);
    squareAY = static_cast<int>(pose.orientation.y);
    squareBX = static_cast<int>(pose.orientation.z);
    squareBY = static_cast<int>(pose.orientation.w);

    squareWidth = squareBX - squareAX;
    squareHeigth = squareBY - squareAY;

    std::cout<<"squareWidth:"<<squareWidth<<std::endl;
    std::cout<<"squareHeigth:"<<squareHeigth<<std::endl;

    //识别的相机坐标
    camX = (squareWidth / 2)+squareAX;
    camY = (squareHeigth / 2) + squareAY;

    std::cout<<"camX:"<<camY<<std::endl;
    std::cout<<"camY:"<<camY<<std::endl;

    //１，识别成功　０，识别失败
    isDetesion = static_cast<bool>(pose.position.x);

    isdone = true;

    data.push_back(camX);
    data.push_back(camY);
    data.push_back(squareAX);
    data.push_back(squareAY);
    data.push_back(squareBX);
    data.push_back(squareBY);
    sendCamPoseData(data);

    return true;
}

void MainWindow::startMove()
{
    boost::function0<void > f = boost::bind(&MainWindow::startMoveThrd, this);
    moveThrd = new boost::thread(f);
    return;
}

void MainWindow::startMoveThrd()
{
    double value = 0;
    int attemp = 0;

    //识别
    detectionThread();

    while(!haveObj && attemp <= 10){
        usleep(300000);
        attemp++;
    }
    if(attemp >= 10){
        emitUIUpdata("<font color = red> 等待超时!!! </font>");
        voiceStatus();
        updataImg();
        return;
    }
    if(!haveObj){
        voiceStatus();
        updataImg();
        return;
    }

    attemp = 0;
    hsc3->setHscR(10,0);
    usleep(200000);
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
        emitUIUpdata("<font color = green> 机器人到位失败!!! </font>");
        voiceStatus();
        updataImg();
        return;
    }
    hsc3->setHscR(11,0);

    emitUIUpdata("<font color = green> 机器人已到位!!! </font>");

    emitUIUpdata("<font color = green> 识别成功，正在抓取!!! </font>");

    do{
        hsc3->getHscR(10,value);
        std::cout<<"start_move"<<std::endl;
        sleep(1.0);
    }
    while(value == 1.0);
    emitUIUpdata("<font color = green> 抓取成功!!! </font>");
    voiceStatus();
    updataImg();
    return;
}

void MainWindow::startMaulModeBnt()
{
    startMove();
    return;
}

void MainWindow::selectObjCombobox(int index)
{
    cv::Mat selMat;
    switch (index) {
    case 0:
        objType = BEAR;
        setReturnStrtoUI("<font color = green > 选择小熊 </font>");
        break;
    case 1:
        objType = RABBIT;
        setReturnStrtoUI("<font color = green > 选择小兔子 </font>");
        break;
    case 2:
        objType = GIRAFFE;
        setReturnStrtoUI("<font color = green > 选择长颈鹿 </font>");
        break;
    default:
        objType = NONE;
        break;
    }

//    if(!getDetectionRobotPose())
//        return;

//    if(isDetesion){
//        if(camOpera->opencvDrawing(squareAX,squareAY,squareBX,squareBX,selMat,camSDeteImgFile)){
//            showImageLabelChange(camOpera->colorImg);
//            return;
//        }
//    }

    return;
}

void MainWindow::LabelDisplayMat(QLabel *label, cv::Mat mat)
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

    label->setScaledContents(true);
    QSize qs = label->rect().size();

    QPixmap qimp = QPixmap::fromImage(Img);
    QPixmap nimp = qimp.scaled(qs);

    label->setPixmap(nimp);

    return;
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

    ui->frame_vision->setGeometry(fx,fy,fw,fh);

    return;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    calDialog->close();
    ttsDialog->close();
    imgDialog->close();
//    QMessageBox::StandardButton button;
//    button=QMessageBox::question(this,tr("退出程序"),QString(tr("确认退出程序")),QMessageBox::Yes|QMessageBox::No);
//    if(button==QMessageBox::No)
//    {
//        event->ignore(); // 忽略退出信号，程序继续进行
//    }
//    else if(button==QMessageBox::Yes)
//    {
//        calDialog->close();
//        event->accept(); // 接受退出信号，程序退出
//    }
    return;
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->label_show_image)//当事件发生在u1（为Qlabel型）控件上
    {
        if (event->type() == QEvent::MouseButtonDblClick)//当为双击事件时
        {
            clikedcount++;
            if (clikedcount % 2 == 0) //此处为双击一次全屏，再双击一次退出
            {
                ui->label_show_image->setWindowFlags(Qt::Dialog);
                ui->label_show_image->showMaximized();//全屏显示
            }
            else
            {
                ui->label_show_image->setWindowFlags(Qt::SubWindow);
                ui->label_show_image->showNormal();//退出全屏
            };

        }
        return QObject::eventFilter(obj, event);
    }
}

void MainWindow::showDetectionRobotData(double rx, double ry)
{
    LocData locdata13, locdata14;
    int Lrindex13 = 13, Lrindex14 = 14;
    if(!hsc3->getHscLR(Lrindex13,locdata13))
    {
        emitUIUpdata("<font color = red> 获取机器人坐标失败!!! </font>");
        haveObj = false;
        return;
    }
    if(!hsc3->getHscLR(Lrindex14,locdata14))
    {
        emitUIUpdata("<font color = red> 获取机器人坐标失败!!! </font>");
        haveObj = false;
        return;
    }

    ui->lineEdit_result_X->setText(QString::number(rx,'f',4));
    ui->lineEdit_result_Y->setText(QString::number(ry,'f',4));
    ui->lineEdit_result_Z->setText(QString::number(locdata14[2],'f',4));
    ui->lineEdit_result_A->setText(QString::number(locdata14[3],'f',4));
    ui->lineEdit_result_B->setText(QString::number(locdata14[4],'f',4));
    ui->lineEdit_result_C->setText(QString::number(locdata14[5],'f',4));

    locdata13[0] = rx;
    locdata13[1] = ry;
    locdata14[0] = rx;
    locdata14[1] = ry;

    if(!hsc3->setHscLR(Lrindex13, locdata13))
    {
        emitUIUpdata("<font color = red> 识别娃娃失败,LR13!!! </font>");
        haveObj = false;
        return;
    }

    if(!hsc3->setHscLR(Lrindex14, locdata14))
    {
        emitUIUpdata("<font color = red> 识别娃娃失败,LR14!!! </font>");
        haveObj = false;
        return;
    }
    return;
}

void MainWindow::detectionDone(bool have, int numB, int numR, int numG, double rcoRate)
{
    if(!have){
        isdone = false;
        emitUIUpdata("<font color = red> 无识别物体!!! </font>");
        return;
    }
    QString strRate = QString::number(rcoRate,'f',4);
    QString strB = QString::number(numB);
    QString strR = QString::number(numR);
    QString strG = QString::number(numG);

    emitUIUpdata("<font color = blue> 识别完成!!! </font>");
    emitUIUpdata("<font color = blue> 识别率："+strRate+" </font>");
    emitUIUpdata("<font color = blue> 小熊：　"+strB+" 个  </font>");
    emitUIUpdata("<font color = blue> 小兔子："+strR+" 个 </font>");
    emitUIUpdata("<font color = blue> 长颈鹿："+strG+" 个 </font>");

    isdone = true;
    return;
}

bool MainWindow::getDetectionRobotPose()
{
    if(!getCamPose(objType)){
        //haveObj = false;
        return false;
    }

    std::cout<<"isDetesion:"<<isDetesion<<std::endl;
    if(!isDetesion){
        std::cout<< "识别失败："<<isDetesion<<std::endl;
        emitUIUpdata("<font color = red> 识别娃娃失败!!! </font>");
        //haveObj = false;
        return false;
    }

    std::cout<<"camX:"<<camX<<std::endl;
    std::cout<<"camY:"<<camY<<std::endl;

    if(!camOpera->getCalibraHomeData())
        return false;

    if(!camOpera->getDetesionResult(camX,camY,detesionRPose))
        return false;
    std::cout<<"detesionRPose:"<<detesionRPose[0]<<std::endl;
    emitDetectionUIUpdata(detesionRPose[0],detesionRPose[1]);

    return true;
}

void MainWindow::voiceStatus()
{
    if(voiceState = 2){
        voice->startVoiceRecognition();
        voiceState = 1;
    }
    return;
}

void MainWindow::showCamPose(LocData data)
{
    ui->label_CX->setText(QString::number(data[0],'f',4));
    ui->label_CY->setText(QString::number(data[1],'f',4));
    ui->label_CZ->setText(QString::number(data[2],'f',4));
    ui->label_CA->setText(QString::number(data[3],'f',4));
    ui->label_CB->setText(QString::number(data[4],'f',4));
    ui->label_CC->setText(QString::number(data[5],'f',4));
    return;
}

void MainWindow::updataImg()
{
    camTakePirtureBnt();
}


void MainWindow::showImageDialog()
{
    imgDialog->show();
    return;
}

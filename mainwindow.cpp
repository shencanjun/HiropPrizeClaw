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
    qRegisterMetaType<std::vector<int> >("std::vector<int> ");

   // MainXmlFile = "/home/fshs/HiropPrizeClaw/build-HiropPrizeClaw-unknown-Debug/MainConfig.xml";

    progName = "HIROP.PRG";
    camImageFileName ="./data/show.jpg";
    camADeteImgFile = "./data/Adetetion.jpg";
    camSDeteImgFile = "./data/Sdetetion.jpg";
    camXmlFileName = "calibrate";
    calibXmlName = "./data/calibrateData.xml";

    objType = BEAR;
    voiceState = 0;
    voiceStep = 0;
    clikedcount = 0;
    stepFlag = false;
    steptime = 100;//10s

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

    objList<<"小熊"<<"小兔子"<<"长颈鹿"<<"小海豚"<<"小豹子";

   // readMainXml();
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
    connect(imgDialog, &ImageDialog::emitCloseSignal, this, &MainWindow::closeImageDialog);
    connect(this, &MainWindow::emitsendsound, this, &MainWindow::soundsignal);

    connect(camOpera, &CamraOperate::emitImagesignal, imgDialog, &ImageDialog::reviceImg);

    connect(showgroupTimer, &QTimer::timeout, this, &MainWindow::setFrameInCenter);

    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
    connect(ui->actionVoiceTts, &QAction::triggered,this,&MainWindow::showTtsMscDialog);
    connect(ui->actiontImage,&QAction::triggered, this, &MainWindow::showImageDialog);
    connect(ui->actionSendProg, &QAction::triggered, this, &MainWindow::SendProgAction);
    connect(ui->pushButton_VoiceRecognition,&QPushButton::clicked,this,&MainWindow::OpenOrCloseVoiceRecognitionBnt);
    connect(ui->pushButton_connectRobot,&QPushButton::clicked,this,&MainWindow::connectHsRobotBnt);
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
    stepFlag = false;
    hscMsgThrdFlag = false;
    hscThredflat = false;
    delete showgroupTimer;
    delete camOpera;
    delete hsc3;
    delete parse;
    delete voice;
//    delete calDialog;
//    delete imgDialog;
//    delete ttsDialog;
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
//    QStringList strlist = str.split("<");
//    if(strlist.size() >= 1){
//        strlist = strlist[1].split(">");
//        if(strlist.size() >= 1){
//            for(int i=0;i<strlist.size();i++)
//            {
//                qDebug()<<strlist[1];
//            }
//            voice->textToSoundPlay(strlist[1]);
//        }
//    }
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
    if(str == "休眠"){
        voiceStep = 0;
    }
    switch (voiceStep) {
    case 0:
        if(str != "嘿华数")
            return;
        voice->textToSoundPlay("你好！欢迎来到华数机器人展馆，请选择你喜欢的动物娃娃，并告诉我...",
                               "./data/tts_data/step0.wav");
        camTakePirtureBnt();
        //sendsound("./data/tts_data/step0.wav");
        voiceSleep(6);
        voiceStep = 1;
        break;
    case 1:
        stepFlag = false;
        if(str== objList[0]){
            objType = BEAR;
        }
        else if(str == objList[1]){
            objType = RABBIT;
        }
        else if(str == objList[2]){
            objType = GIRAFFE;
        }
        else if(str == objList[3]){
            objType = DOLPHIN;
        }
        else if(str == objList[4]){
            objType = LEOPARD;
        }
        else{
            break;
        }
        voice->textToSoundPlay("你的选择为："+ str +"请您确认","./data/tts_data/step1.wav");
        //sendsound("./data/tts_data/step1.wav");
        voiceSleep(3);

        emitUIUpdata("<font color = green>你的选择为：</font> <font color = blue size = 10>"+ str +" </font> <font color = green size = 5> 请你确认</font>");
        voiceStep = 3;
        break;
    case 2:
        stepFlag = false;
        if(str == "确认"){
            emitUIUpdata("<font color = green>当前选择的娃娃为：</font> <font color = blue size = 10> 小兔子  </font> <font color = green size = 5>请你再次确认</font>");
            //voice->textToSoundPlay("当前选择的娃娃为：小兔子 请再次确认","./data/tts_data/step2_y.wav");
            voiceSleep(4);
            voiceStep = 3;
        }
        else if(str == "不确定"){
            emitUIUpdata("<font color = green>请重新选择你喜欢动物娃娃</font>");
            voice->textToSoundPlay("请重新选择你喜欢动物娃娃","./data/tts_data/step2_n.wav");
            //sendsound("./data/tts_data/step2_n.wav");
            voiceSleep(3);
            voiceStep = 1;
        }
        break;
    case 3:
        stepFlag = false;
        if(str == "确认")
        {
            voice->stopVoiceRecognition();
            if(objType == BEAR){
                emitUIUpdata("抓取：<font color = blue size = 10> 小熊 </font>" );
                emitUIUpdata("<font color = green>正在为您抓取小熊</font>");
               // voice->textToSoundPlay("正在为查找查找小熊","./data/tts_data/step3_b.wav");
                //sendsound("./data/tts_data/step3_b.wav");
                sleep(1.0);
                selectObjCombobox(0);
            }
            else if(objType == RABBIT){
                setReturnStrtoUI("抓取：<font color = blue size = 10> 小兔子 </font>" );
                emitUIUpdata("<font color = green>正在为您抓取小兔子</font>");
                //voice->textToSoundPlay("正在查找小兔子","./data/tts_data/step3_r.wav");
                //sendsound("./data/tts_data/step3_r.wav");
                sleep(1.0);
                selectObjCombobox(1);
            }
            else if(objType == GIRAFFE){
                setReturnStrtoUI("抓取：<font color = blue size = 10> 长颈鹿 </font>" );
                emitUIUpdata("<font color = green>正在为您抓取长颈鹿</font>");
                //voice->textToSoundPlay("正在查找长颈鹿","./data/tts_data/step3_g.wav");
                //sendsound("./data/tts_data/step3_g.wav");
                sleep(1.0);
                selectObjCombobox(2);
            }
            else if(objType == DOLPHIN){
                setReturnStrtoUI("抓取：<font color = blue size = 10> 小海豚 </font>" );
                emitUIUpdata("<font color = green>正在为您抓取小海豚</font>");
                //voice->textToSoundPlay("正在查找小海豚","./data/tts_data/step3_d.wav");
                //sendsound("./data/tts_data/step3_g.wav");
                sleep(1.0);
                selectObjCombobox(3);
            }
            else if(objType == LEOPARD){
                setReturnStrtoUI("抓取：<font color = blue size = 10> 小豹子 </font>" );
                emitUIUpdata("<font color = green>正在为您抓取小豹子</font>");
                //voice->textToSoundPlay("正在查找小豹子","./data/tts_data/step3_l.wav");
                //sendsound("./data/tts_data/step3_l.wav");
                sleep(1.0);
                selectObjCombobox(4);
            }
            else{
                return;
            }
            voiceState = 2;
            startMove();
            voiceStep = 0;
        }
        else if(str == "不确认")
        {
            emitUIUpdata("<font color = green>请重新选择你喜欢动物娃娃</font>");
            //voice->textToSoundPlay("请重新选择你喜欢动物娃娃","./data/tts_data/step3_n.wav");
            sendsound("./data/tts_data/step3_n.wav");
            voiceStep = 1;
            //emitUIUpdata("voiceStep = 1");
        }
        break;
    default:
        //voiceStep = 0;
        break;
    }
    return;
    //QString pstr = ""+str;
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
                //initRobot();
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
    bool en = false;

    if(!HscStatus)
    {
        ui->lineEdit_Msg->setStyleSheet("background-color: rgb(255, 85, 0);");
    }
    if(hsc3->getHscEnanle(en))
    {
        if(en){
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
        }
    }

    return;

//    ProgInfo info;
//    if(hsc3->getHscProInfo(progName, info))
//    {
//        if(info.state == AUTO_STATE_RUNNING)
//        {
//            ui->pushButton_start->setText("停止");
//            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
//        }
//    }
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
            sleep(1);
            initRobot();
            hscMsgThrdFlag = true;
            hscThredflat = true;
            getHscLocDataStart();
            getHscMsgThrd();
            emitUIUpdata("<font color = green> 连接机器人成功！！！</font>");
         }
         else
         {
            ui->lineEdit_rip->setReadOnly(false);
            ui->lineEdit_rport->setReadOnly(false);
            emitUIUpdata("<font color = red> 连接机器人失败！！！ </font>");
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
            hscThredflat = false;
            hscMsgThrdFlag = false;
            emitUIUpdata("<font color = green> 断开连接机器人成功！！！</font>");
        }
        else
        {
           ui->lineEdit_rip->setReadOnly(true);
           ui->lineEdit_rport->setReadOnly(true);
           emitUIUpdata("<font color = red> 断开连接机器人失败！！！ </font>");
        }
    }
    return;
}

/*
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
*/

void MainWindow::HsRobotStartBnt()
{
    if(ui->pushButton_start->text() == "开始" )
    {
        hsc3->setHscMode(OP_AUT);
        hsc3->setHscVord(80);
        if(hsc3->HscLoadPRG(progName) && HscStatus){
            emitUIUpdata("<font color = green> 机器人加载程序成功！！！</font>");
        }
        else{
            emitUIUpdata("<font color = red> 机器人加载程序失败！！！ </font>");
            return;
        }

        if(hsc3->setHscEnanle(true) && HscStatus){
            emitUIUpdata("<font color = green> 机器人使能成功！！！</font>");
        }
        else{
            emitUIUpdata("<font color = red> 机器人使能失败！！！ </font>");
            return;
        }
        sleep(1.0);
        if(hsc3->HscPrgStart(progName) && HscStatus){
            ui->pushButton_start->setText("停止");
            ui->pushButton_start->setStyleSheet("background-color: rgb(85, 255, 127)");
            emitUIUpdata("<font color = green> 开始运行机器人程序成功！！！</font>");
        }
        else
        {
            emitUIUpdata("<font color = red> 开始运行机器人程序失败！！！ </font>");
            return;
        }
    }
    else
    {
        if(hsc3->HscUnloadPRG(progName)){
            emitUIUpdata("<font color = green> 机器人卸载程序成功！！！</font>");
        }
        else{
            setReturnStrtoUI("<font color = red> 机器人卸载程序失败！！！ </font>");
        }

        if(hsc3->setHscEnanle(false)){
            emitUIUpdata("<font color = green> 机器人失能成功！！！<font>");
        }
        else{
            emitUIUpdata("<font color = red> 机器人使能失败！！！ </font>");
        }
        sleep(1.0);
        if(hsc3->HscPrgStop(progName)){
            ui->pushButton_start->setText("开始");
            ui->pushButton_start->setStyleSheet("background-color: rgb(255, 255, 255)");
            emitUIUpdata("<font color = green> 停止运行机器人程序成功！！！</font>");
        }
        else{
            emitUIUpdata("<font color = red> 停止运行机器人程序失败！！！ </font>");
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
    delete HscLocThrd;
    return;
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

void MainWindow::getHscMsgThrd()
{
    boost::function0<void > f = boost::bind(&MainWindow::HscMsgStatusLET, this);
    hscMsgThrd = new boost::thread(f);
    delete hscMsgThrd;
    return;
}

void MainWindow::HscMsgStatusLET()
{
    ErrLevel level;
    std::string msg;
    QString qstr;

    while(hscMsgThrdFlag){

        if(!hsc3->getFaultMessage(level,msg)){
            emitUIUpdata("<font color = red> 获取机器人报警失败！！！ </font>");
            continue;
        }
        qstr = QString::fromStdString(msg);
        //if(msg == "")
        //    return;
        if(level == ERR_LEVEL_ERROR || level == ERR_LEVEL_FATAL){
            HscStatus = false;
            ui->lineEdit_Msg->setStyleSheet("background-color: rgb(255, 85, 0);");

            emitUIUpdata("<font color = red>" + qstr + "</font>");
        }
        else if(level == ERR_LEVEL_WARNING){
            emitUIUpdata("<font color = yellow>" + qstr + "</font>");
        }
        else{
            ;//emitUIUpdata("<font color = black>" + qstr + "</font>");
        }
        usleep(500000);
    }

    return;
}

void MainWindow::HscClearFaultBnt()
{
    if(!hsc3->HscClearFault())
    {
        emitUIUpdata("<font color = red> 机器人复位失败！！！ </font>");
        return;
    }
    HscStatus = true;
    ui->lineEdit_Msg->setStyleSheet("background-color: rgba(255, 255, 255, 80);");
    emitUIUpdata("<font color = green> 机器人复位成功！！！ </font>");
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
    draw.release();
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
    delete thrd;
    return;
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
            //voice->textToSoundPlay("没有小熊了!","./data/tts_data/step_nb.wav");
            sendsound("./data/tts_data/step_nb.wav");
            return false;
        }
        //voice->textToSoundPlay("正在抓取小熊!","./data/tts_data/step_yb.wav");
        sendsound("./data/tts_data/step_yb.wav");
        pose = camOpera->vecBear[0];
        std::cout<<"BEAR"<<std::endl;
    }
    else if(type == RABBIT){
        if(camOpera->vecRabbit.size() <= 0){
            emitUIUpdata("<font color = red> 没有小兔子了!!! </font>");
            //voice->textToSoundPlay("没有小兔子了!","./data/tts_data/step_nr.wav");
            sendsound("./data/tts_data/step_nr.wav");
            return false;
        }
        //voice->textToSoundPlay("正在抓取小兔子!","./data/tts_data/step_yr.wav");
        sendsound("./data/tts_data/step_yr.wav");
        pose = camOpera->vecRabbit[0];
        std::cout<<"RABBIT"<<std::endl;
    }
    else if(type == GIRAFFE){
        if(camOpera->vecGiraffe.size() <= 0){
            emitUIUpdata("<font color = red> 没有长颈鹿了!!! </font>");
            //voice->textToSoundPlay("没有长颈鹿了!","./data/tts_data/step_ng.wav");
            sendsound("./data/tts_data/step_ng.wav");
            return false;
        }
        //voice->textToSoundPlay("正在抓取长颈鹿!","./data/tts_data/step_yg.wav");
        sendsound("./data/tts_data/step_yg.wav");
        pose = camOpera->vecGiraffe[0];
        std::cout<<"GIRAFFE"<<std::endl;
    }
    else if(type == DOLPHIN){
        if(camOpera->vecDolphin.size() <= 0){
            emitUIUpdata("<font color = red> 没有小海豚了!!! </font>");
            //voice->textToSoundPlay("没有小海豚了!","./data/tts_data/step_nd.wav");
            sendsound("./data/tts_data/step_nd.wav");
            return false;
        }
        //voice->textToSoundPlay("正在抓取小海豚!","./data/tts_data/step_yd.wav");
        sendsound("./data/tts_data/step_yd.wav");
        pose = camOpera->vecDolphin[0];
        std::cout<<"GIRAFFE"<<std::endl;
    }
    else if(type == LEOPARD){
        if(camOpera->vecLeopard.size() <= 0){
            emitUIUpdata("<font color = red> 没有小豹子了!!! </font>");
            //voice->textToSoundPlay("没有小豹子了!","./data/tts_data/step_nl.wav");
            sendsound("./data/tts_data/step_nl.wav");
            return false;
        }
        //voice->textToSoundPlay("正在抓取小豹子!","./data/tts_data/step_yl.wav");
        sendsound("./data/tts_data/step_yl.wav");
        pose = camOpera->vecLeopard[0];
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
    if(type == GIRAFFE){
        camY = (squareHeigth / 2) + squareAY + 15;
    }
    if(type == LEOPARD){
        camY = (squareHeigth / 2) + squareAY + 10;
    }

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

    while(!haveObj && attemp <= 1){
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
    voice->textToSoundPlay("识别成功，正在抓取!!!","./data/tts_data/step_pick.wav");
    //sendsound("./data/tts_data/step_pick.wav");
    do{
        hsc3->getHscR(10,value);
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
    //cv::Mat selMat;
    switch (index) {
    case 0:
        objType = BEAR;
        emitUIUpdata("<font color = green > 选择小熊 </font>");
        break;
    case 1:
        objType = RABBIT;
        emitUIUpdata("<font color = green > 选择小兔子 </font>");
        break;
    case 2:
        objType = GIRAFFE;
        emitUIUpdata("<font color = green > 选择长颈鹿 </font>");
        break;
    case 3:
        objType = DOLPHIN;
        emitUIUpdata("<font color = green > 选择小海豚 </font>");
        break;
    case 4:
        objType = LEOPARD;
        emitUIUpdata("<font color = green > 选择小豹子 </font>");
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

    Rgb.release();
    mat.release();

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
            imgDialog->showMaximized();
            //clikedcount++;
            if (clikedcount % 2 == 0) //此处为双击一次全屏，再双击一次退出
            {
                //ui->label_show_image->setWindowFlags(Qt::Dialog);
                //ui->label_show_image->showMaximized();//全屏显示

            }
            else
            {
                //imgDialog->close();
                //ui->label_show_image->setWindowFlags(Qt::SubWindow);
                //ui->label_show_image->showNormal();//退出全屏
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

void MainWindow::detectionDone(bool have, std::vector<int> num, double rcoRate)
{
    if(!have){
        isdone = false;
        emitUIUpdata("<font color = red> 无识别物体!!! </font>");
        return;
    }
    QString strRate = QString::number(rcoRate,'f',4);
    QString strB = QString::number(num[0]);
    QString strR = QString::number(num[1]);
    QString strG = QString::number(num[2]);
    QString strD = QString::number(num[3]);
    QString strL = QString::number(num[4]);

    emitUIUpdata("<font color = blue> 识别完成!!! </font>");
    emitUIUpdata("<font color = blue> 识别率："+strRate+" </font>");
    emitUIUpdata("<font color = blue> 小熊：　"+strB+" 个  </font>");
    emitUIUpdata("<font color = blue> 小兔子："+strR+" 个 </font>");
    emitUIUpdata("<font color = blue> 长颈鹿："+strG+" 个 </font>");
    emitUIUpdata("<font color = blue> 小海豚："+strD+" 个 </font>");
    emitUIUpdata("<font color = blue> 小豹子："+strL+" 个 </font>");

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
        voiceStep = 0;
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


void MainWindow::SendProgAction()
{
    QString file = QFileDialog::getOpenFileName(this,"选择主程序","../HiropPrizeClaw/","PRG(*.PRG)");
    if(file.isEmpty())
        return;

    if(!hsc3->sendProg(file)){
        emitUIUpdata("<font color = red> 发送主程序失败!!! </font>");
        return;
    }
    emitUIUpdata("<font color = green> 发送主程序成功!!! </font>");
    return;
}


void MainWindow::voiceStepReset()
{
    boost::function0 <void > f = boost::bind(&MainWindow::voiceSteoResetThrd, this);
    setpThrd = new boost::thread(f);
    delete setpThrd;
    return;
}

void MainWindow::voiceSteoResetThrd()
{
    int count = 0;
    while(stepFlag && count <= steptime){
        usleep(100000);
        count++;
    }
    if(count >= steptime){
        voiceStep = 0;
        stepFlag = false;
    }
    return;
}

void MainWindow::closeImageDialog()
{
    std::cout<<"revice revice"<<std::endl;
    imgDialog->close();
    return;
}


void MainWindow::voiceSleep(int time)
{
    boost::function<void (int)> f = boost::bind(&MainWindow::voiceSleepThrd, this, -1);
    vsThrd = new boost::thread(boost::bind(f,time));
    delete vsThrd;
    return;
}

void MainWindow::voiceSleepThrd(int time)
{
    voice->stopVoiceRecognition();
    int i = 0;
    while(i < time){
        i++;
        //usleep(1000000);
        sleep(1);
    }
    std::cout<<"start"<<std::endl;
    voice->startVoiceRecognition();
}

void MainWindow::soundsignal(QString fileName)
{
    QSound *sound = new QSound(fileName);
    connect(this, SIGNAL(emitsendsoundThrd(void)), sound, SLOT(play(void)));
    sendsoundThrd();
    disconnect(this, SIGNAL(emitsendsoundThrd(void)), sound, SLOT(play(void)));
    sound->fileName().clear();
    //delete sound;
    return;
}

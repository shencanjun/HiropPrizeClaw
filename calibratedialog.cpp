#include "calibratedialog.h"
#include "ui_calibratedialog.h"

CalibrateDialog::CalibrateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CalibrateDialog)
{
    ui->setupUi(this);
    raw = 0;
    ui->tableWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    tableMenu = new QMenu(ui->tableWidget);
    tableMenu->setStyleSheet("background-color: rgb(238, 238, 236);color: rgb(46, 52, 54);");

    action = new QAction("撤回", this);
    parse = new ParseConfig();
    recordTimer = new QTimer();
    deleteTimer = new QTimer();

    connect(action, &QAction::triggered, this, &CalibrateDialog::slotActionDelete);
    connect(ui->pushButton_startCalib, &QPushButton::clicked, this, &CalibrateDialog::startCalibration);
    connect(ui->pushButton_record,&QPushButton::clicked,this,&CalibrateDialog::recordCalibrateData);
    connect(ui->pushButton_clibrate,&QPushButton::clicked, this, &CalibrateDialog::getCalibrationResult);
    connect(ui->tableWidget, &QTableWidget::customContextMenuRequested,
            this, &CalibrateDialog::on_tableWidget_customContextMenuRequested);
    connect(ui->pushButton_save, &QPushButton::clicked, this, &CalibrateDialog::SaveCalibration);
    connect(recordTimer, &QTimer::timeout, this, &CalibrateDialog::startHscCalibrate);
    connect(deleteTimer, &QTimer::timeout, this, &CalibrateDialog::HscCalibrateDelete);
}

CalibrateDialog::~CalibrateDialog()
{
    delete ui;
}

void CalibrateDialog::setOpeaObject(HSC3ROBOT *MWhsc3, CamraOperate *cam)
{
    Chsc3 = MWhsc3;
    camCalib = cam;
    return;
}

void CalibrateDialog::readCalibrateData()
{
    double acc, comx, comy;
    std::cout<<"calibDataFileName:"<<calibDataFileName.toStdString()<<std::endl;
    parse->readCalibXML(calibDataFileName,acc,comx,comy);
    ui->label_accuary->setText(QString::number(acc,'f',4));
    ui->lineEdit_compensateX->setText(QString::number(comx,'f',4));
    ui->lineEdit_compensateY->setText(QString::number(comy,'f',4));
    return;
}

void CalibrateDialog::startCalibration()
{
    if(!camCalib->startEyeCalirating())
    {
        QMessageBox::warning(NULL, "提示", "<font color = red >开始标定失败!!!</font>", QMessageBox::Yes);
    }

    camCalib->getCalibrateImage();

    float cx ,cy;
    for(int i=0;i<9;i++)
    {
        camCalib->readEyeData(cx, cy, i);
        ui->tableWidget->setItem(i, cColumnX, new QTableWidgetItem(QString::number(cx, 'f', 4)));
        ui->tableWidget->setItem(i, cColumnY, new QTableWidgetItem(QString::number(cy, 'f', 4)));
    }
    recordTimer->start(500);
    deleteTimer->start(500);
    return;
}

void CalibrateDialog::recordCalibrateData()
{
    LocData data;
    std::vector<double> calData;
    if(!Chsc3->getHscLoc(data)){
        std::cout<<"获取机器人笛卡尔坐标失败！！！"<<std::endl;
        QMessageBox::warning(NULL, "提示", "<font color = red >获取机器人笛卡尔坐标失败！！！</font>", QMessageBox::Yes);
        return;
    }
    if(data.size() < 2){
        std::cout<<"机器人笛卡尔坐标错误！！！"<<std::endl;
        QMessageBox::warning(NULL, "提示", "<font color = red >机器人笛卡尔坐标错误！！！</font>", QMessageBox::Yes);
        return;
    }
    if(data.size() <= 2)
        return;

    ui->tableWidget->setItem(raw, rColumnX, new QTableWidgetItem(QString::number(data[0], 'f', 4)));
    ui->tableWidget->setItem(raw, rColumnY, new QTableWidgetItem(QString::number(data[1], 'f', 4)));

    calData.push_back(data[0]);
    calData.push_back(data[1]);

    calibraData.push_back(calData);
    std::cout<<"raw:"<<raw<<std::endl;
    raw++;
    if(raw >= 9)
        raw = 9;

    return;
}

void CalibrateDialog::getCalibrationResult()
{
    double accu;
    if(calibraData.size()<9){
        QMessageBox::warning(NULL, "提示", "<font color = red >数据不完整,\n请记录完整后再进行标定</font>", QMessageBox::Ok);
        return;
    }
    recordTimer->stop();
    deleteTimer->stop();
    for(int i = 0; i < calibraData.size(); i++){
        if(!camCalib->writeCalibrateData(calibraData[i][0],calibraData[i][1],i)){
            return;
        }
    }
    if(!camCalib->getCalibrateResult(accu))
    {
        QMessageBox::warning(NULL, "提示", "<font color = red >标定失败!!!</font>", QMessageBox::Yes);
    }
    std::cout<<"accu :"<<accu<<std::endl;
    ui->label_accuary->setText(QString::number(accu, 'f', 4));
    return;
}

void CalibrateDialog::on_tableWidget_customContextMenuRequested(const QPoint &pos)
{
    tableMenu->addAction(action);
    tableMenu->exec(QCursor::pos());

    return;
}

void CalibrateDialog::slotActionDelete()
{
    std::cout<<"delete"<<std::endl;
    if(calibraData.size() <= 0)
        return;
    calibraData.pop_back();
    raw--;
    ui->tableWidget->setItem(raw, rColumnX, new QTableWidgetItem(""));
    ui->tableWidget->setItem(raw, rColumnY, new QTableWidgetItem(""));
    //QMessageBox::warning(NULL, "提示", "<font color = red >是否确认撤回数据？</font>", QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    std::cout<<"raw:"<<raw<<std::endl;
    if(raw <= 0)
        raw = 0;
    return;
}

void CalibrateDialog::SaveCalibration()
{
    parse->writeXmlFile(calibDataFileName,
                        ui->label_accuary->text().toDouble(),
                        ui->lineEdit_compensateX->text().toDouble(),
                        ui->lineEdit_compensateY->text().toDouble());
    sendCom();
    return;
}

void CalibrateDialog::startHscCalibrate()
{
    bool io;
    if(Chsc3->getHscIoValue(28,io)){
        if(io){
            recordCalibrateData();
            Chsc3->setHscIoValue(28,false);
        }
        else{
            ;
        }
    }
    else {
        std::cout<<"获取机器人ＩＯ值失败"<<std::endl;
    }
    return;
}

void CalibrateDialog::HscCalibrateDelete()
{
    bool io;
    if(Chsc3->getHscIoValue(29,io)){
        if(io){
            slotActionDelete();
            Chsc3->setHscIoValue(29,false);
        }
        else{
            ;
        }
    }
    else {
        std::cout<<"获取机器人ＩＯ值失败"<<std::endl;
    }
    return;
}



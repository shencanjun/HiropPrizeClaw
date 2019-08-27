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
    tableMenu->setStyleSheet("background-color: rgb(255, 255, 255)");
    action = new QAction("删除", this);
    parse = new ParseConfig();
    connect(action, &QAction::triggered, this, &CalibrateDialog::slotActionDelete);
    connect(ui->pushButton_startCalib, &QPushButton::clicked, this, &CalibrateDialog::startCalibration);
    connect(ui->pushButton_record,&QPushButton::clicked,this,&CalibrateDialog::recordCalibrateData);
    connect(ui->pushButton_clibrate,&QPushButton::clicked, this, &CalibrateDialog::getCalibrationResult);
    connect(ui->tableWidget, &QTableWidget::customContextMenuRequested,
            this, &CalibrateDialog::on_tableWidget_customContextMenuRequested);
    connect(ui->pushButton_save, &QPushButton::clicked, this, &CalibrateDialog::SaveCalibration);
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
    return;
}

void CalibrateDialog::recordCalibrateData()
{
    LocData data;
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

    hscLocData = data;
    if(!camCalib->writeCalibrateData(data[0],data[1],raw))
    {
        return;
    }
    //std::cout<<"raw:" <<raw<<std::endl;

    ui->tableWidget->setItem(raw, rColumnX, new QTableWidgetItem(QString::number(data[0], 'f', 4)));
    ui->tableWidget->setItem(raw, rColumnY, new QTableWidgetItem(QString::number(data[1], 'f', 4)));

    raw++;
    if(raw >= 9)
        raw = 0;

    return;
}

void CalibrateDialog::getCalibrationResult()
{
    double accu;
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
    QMessageBox::warning(NULL, "提示", "<font color = red >是否确认删除本列数据？</font>", QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    return;
}

void CalibrateDialog::SaveCalibration()
{
    parse->writeXmlFile(calibDataFileName,
                        ui->label_accuary->text().toDouble(),
                        ui->lineEdit_compensateX->text().toDouble(),
                        ui->lineEdit_compensateY->text().toDouble());

    return;
}



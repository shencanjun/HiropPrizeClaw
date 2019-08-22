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
    Calib2D = new EyeCalib2D();
    connect(action, &QAction::triggered, this, &CalibrateDialog::slotActionDelete);
    connect(ui->pushButton_record,&QPushButton::clicked,this,&CalibrateDialog::RecordCalibrateData);
    connect(ui->tableWidget, &QTableWidget::customContextMenuRequested, this, &CalibrateDialog::on_tableWidget_customContextMenuRequested);
}

CalibrateDialog::~CalibrateDialog()
{
    delete ui;
}

void CalibrateDialog::setHsc3Object(HSC3ROBOT *MWhsc3)
{
    Chsc3 = MWhsc3;
    return;
}

void CalibrateDialog::RecordCalibrateData()
{
    LocData data;
    std::vector<float> eyeCalib;
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

    Calib2D->readEyeCalib(eyeCalib,raw);
    if(eyeCalib.size() <= 0)
        return;

    hscLocData = data;

    ui->tableWidget->setItem(raw, cColumnX, new QTableWidgetItem(QString::number(eyeCalib[0])));
    ui->tableWidget->setItem(raw, cColumnY, new QTableWidgetItem(QString::number(eyeCalib[1])));
    ui->tableWidget->setItem(raw, rColumnX, new QTableWidgetItem(QString::number(data[0],'f',4)));
    ui->tableWidget->setItem(raw, rColumnY, new QTableWidgetItem(QString::number(data[1],'f',4)));
    Calib2D->writeEyeCalib(eyeCalib[0], eyeCalib[1],data[0], data[1], raw);

    raw++;
    if(raw >= 9)
        raw = 0;

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

void CalibrateDialog::getCalibrationResult()
{
    const std::string fileName = "calibration.xml";
    double accuary;
    Calib2D->estimaHom2D(fileName,accuary);
    ui->label_accuary->setText(QString::number(accuary,'f',4));
    return;
}

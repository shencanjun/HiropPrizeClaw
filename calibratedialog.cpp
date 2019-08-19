#include "calibratedialog.h"
#include "ui_calibratedialog.h"

CalibrateDialog::CalibrateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CalibrateDialog)
{
    ui->setupUi(this);
    raw = 0;
    connect(ui->pushButton_record,&QPushButton::clicked,this,&CalibrateDialog::RecordCalibrateData);
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
    if(!Chsc3->getHscLoc(data)){
        std::cout<<"获取机器人笛卡尔坐标失败！！！"<<std::endl;
        QMessageBox::warning(NULL, "提示", "<font color = red >获取机器人笛卡尔坐标失败！！！</font>", QMessageBox::Yes);
            //return;
    }
    if(data.size() < 2){
        std::cout<<"机器人笛卡尔坐标错误！！！"<<std::endl;
        QMessageBox::warning(NULL, "提示", "<font color = red >机器人笛卡尔坐标错误！！！</font>", QMessageBox::Yes);
            return;
    }

    hscLocData = data;

    ui->tableWidget->setItem(raw, rColumnX, new QTableWidgetItem(QString::number(data[0],'f',4)));
    ui->tableWidget->setItem(raw, rColumnY, new QTableWidgetItem(QString::number(data[2],'f',4)));

    raw++;

    return;
}

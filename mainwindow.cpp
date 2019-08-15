#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->actionCalibrate,&QAction::triggered,this,&MainWindow::showClibrateDialog);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showClibrateDialog()
{
    calDialog = new CalibrateDialog();
    calDialog->setWindowTitle("标定");
    calDialog->show();
}

void MainWindow::connectHsRobotBnt()
{

}


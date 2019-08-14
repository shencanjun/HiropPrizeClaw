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
   // calibrateDialogLayout = new QLayout();
    calibrateDialog = new QDialog();
    calibrateDialog->resize(50,100);
    QPushButton *calibrateBtn = new QPushButton("标定",calibrateDialog);
    QLabel *

    calibrateDialog->show();
}


#include "mainwindow.h"
#include <QApplication>
#include <QDialog>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"HiropPrizeClaw");
    ros::NodeHandle nPrize;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    QApplication a(argc, argv);
    MainWindow w;
    w.sendNodeHanle(nPrize);
    w.setWindowTitle("HiropPrizeClaw");
    w.show();

    return a.exec();
}

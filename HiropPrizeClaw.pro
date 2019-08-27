#-------------------------------------------------
#
# Project created by QtCreator 2019-08-14T11:15:11
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HiropPrizeClaw
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS _LINUX_


SOURCES += main.cpp\
    camraoperate.cpp \
        mainwindow.cpp \
    calibratedialog.cpp \
    parseconfig.cpp \
    voicerecognite.cpp \
    hsc3robot.cpp

HEADERS  += mainwindow.h \
    calibratedialog.h \
    camraoperate.h \
    parseconfig.h \
    voicerecognite.h \
    hsc3robot.h \
    3rdparty/include/CommApi.h

FORMS    += mainwindow.ui \
    calibratedialog.ui

INCLUDEPATH += ./ \
               /home/ros/catkin_ws/devel/include \
               /opt/ros/kinetic/include \
               3rdparty/include \
               /usr/local/include/eigen3 \
               /opt/ros/kinetic/include/opencv-3.3.1-dev


LIBS+= -L/opt/ros/kinetic/lib -lroscpp -lroscpp_serialization -lroslib -lrosconsole -lcv_bridge
LIBS+= -L/usr/local/lib -ljsoncpp -lboost_system -lHandEye
LIBS+= -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_core3 -lopencv_imgcodecs3 -lopencv_imgproc3

#LIBS+= -L/home/fshs/work/HiropPrizeClaw/3rdparty/lib/HsApi -lCommApi -lHsc3Api -lLogApi
LIBS+= -L/home/ros/HiropPrizeClaw/HiropPrizeClaw/3rdparty/lib/HsApi -lCommApi -lHsc3Api -lLogApi

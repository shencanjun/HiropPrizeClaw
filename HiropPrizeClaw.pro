#-------------------------------------------------
#
# Project created by QtCreator 2019-08-14T11:15:11
#
#-------------------------------------------------

QT       += core gui
QT       += multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HiropPrizeClaw
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS _LINUX_


SOURCES += main.cpp\
    camraoperate.cpp \
    imagedialog.cpp \
        mainwindow.cpp \
    calibratedialog.cpp \
    parseconfig.cpp \
    ttsmscdialog.cpp \
    voicerecognite.cpp \
    hsc3robot.cpp

HEADERS  += mainwindow.h \
    3rdparty/ttsmsc/include/ttsmsc.h \
    calibratedialog.h \
    camraoperate.h \
    imagedialog.h \
    parseconfig.h \
    ttsmscdialog.h \
    voicerecognite.h \
    hsc3robot.h \
    3rdparty/include/CommApi.h

FORMS    += mainwindow.ui \
    calibratedialog.ui \
    imagedialog.ui \
    ttsmscdialog.ui

INCLUDEPATH += ./ \
               /home/ros/catkin_ws/devel/include \
               /opt/ros/kinetic/include \
               3rdparty/include \
               /usr/local/include/eigen3 \
               /opt/ros/kinetic/include/opencv-3.3.1-dev \
               3rdparty/ttsmsc/include \
               3rdparty/ttsmsc/include/include


LIBS+= -L/opt/ros/kinetic/lib -lroscpp -lroscpp_serialization -lroslib -lrosconsole -lcv_bridge
LIBS+= -L/usr/local/lib -ljsoncpp -lboost_system -lHandEye
LIBS+= -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_core3 -lopencv_imgcodecs3 -lopencv_imgproc3
LIBS+= -L/usr/lib/x86_64-linux-gnu -lboost_thread

#LIBS+= -L/home/fshs/work/HiropPrizeClaw/3rdparty/ttsmsc/lib -lttsmsc -lmsc_tts
#LIBS+= -L/home/fshs/work/HiropPrizeClaw/3rdparty/lib/HsApi -lCommApi -lHsc3Api -lLogApi

LIBS+= -L/home/ros/HiropPrizeClaw/HiropPrizeClaw/3rdparty/ttsmsc/lib -lttsmsc -lmsc_tts
LIBS+= -L/home/ros/HiropPrizeClaw/HiropPrizeClaw/3rdparty/lib/HsApi -lCommApi -lHsc3Api -lLogApi

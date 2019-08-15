#-------------------------------------------------
#
# Project created by QtCreator 2019-08-14T11:15:11
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HiropPrizeClaw
TEMPLATE = app
#QMAKE_CXXFLAGS += -std=c++11
#QMAKE_LFLAGS += -std=c++11


SOURCES += main.cpp\
        mainwindow.cpp \
    calibratedialog.cpp \
    voicerecognite.cpp

HEADERS  += mainwindow.h \
    calibratedialog.h \
    voicerecognite.h

FORMS    += mainwindow.ui \
    calibratedialog.ui

INCLUDEPATH += ./ \
               /home/ros/catkin_ws/devel/include \
               /opt/ros/kinetic/include

LIBS+= -L/opt/ros/kinetic/lib -lroscpp -lroscpp_serialization -lroslib -lrosconsole
LIBS+= -L/usr/local/lib -ljsoncpp -lboost_system

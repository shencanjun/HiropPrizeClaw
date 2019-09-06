#ifndef VOICERECOGNITE_H
#define VOICERECOGNITE_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hirop_msgs/StartListen.h>
#include <hirop_msgs/StopListen.h>
#include "jsoncpp/json/json.h"
#include <boost/thread.hpp>
#include <ttsmsc.h>
#include <qdebug.h>
#include <QSound>
#include <QFile>

enum ObjType{
    BEAR = 0,
    RABBIT = 1,
    GIRAFFE = 2,
    DOLPHIN = 3,
    LEOPARD = 4,
    NONE,
};

class VoiceRecognite : public QObject
{
    Q_OBJECT

public:
    VoiceRecognite(ros::NodeHandle n);
    ~VoiceRecognite();

private:
    void listenVoice_callback(const std_msgs::String::ConstPtr &msg);
    void parseIntent(std::string &data);

public:
    void send(QString str) const{
        emit emitResultStr(str);
    }

signals:
    void emitResultStr(QString str) const;

public:
    int startVoiceRecognition();
    int stopVoiceRecognition();
    int textToSoundPlay(QString text,QString fileName = "./sound.wav");
    qint64 getAudioTime(const QString &filePath);

public:
    Ttsmsc *msc;
    ros::NodeHandle nVoice;
    ros::ServiceClient clientSatrtListener;
    ros::ServiceClient clientStopListener;
    ros::Subscriber subUserIntent;

    std::string intentFormMsgstr;

    boost::thread *thrdVoice;
};

#endif // VOICERECOGNITE_H

#ifndef VOICERECOGNITE_H
#define VOICERECOGNITE_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hirop_msgs/StartListen.h>
#include <hirop_msgs/StopListen.h>
#include "jsoncpp/json/json.h"
#include <boost/thread.hpp>

enum ObjType{
    DOG = 0,
    RABBIT = 1,
    MONKEY = 2,
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

public:
    ros::NodeHandle nVoice;
    ros::ServiceClient clientSatrtListener;
    ros::ServiceClient clientStopListener;
    ros::Subscriber subUserIntent;

    std::string intentFormMsgstr;

    boost::thread *thrdVoice;
    //boost::function0<void> voiceFun;
};

#endif // VOICERECOGNITE_H

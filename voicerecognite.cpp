#include "voicerecognite.h"

VoiceRecognite::VoiceRecognite(ros::NodeHandle n)
{
    nVoice = n;
    msc = new Ttsmsc();
}

VoiceRecognite::~VoiceRecognite()
{
    //delete msc;
}

void VoiceRecognite::listenVoice_callback(const std_msgs::String::ConstPtr& msg)
{
   if(msg->data == "")
   {
       std::cout<<"msg is empty!!!!"<<std::endl;
       return;
   }

   intentFormMsgstr = msg->data;

   std::cout<<"msg.data:"<<intentFormMsgstr<<std::endl;

   if(intentFormMsgstr == "wakeup")
       send("嘿华数");
   if(intentFormMsgstr == "sleep")
       send("休眠");

   parseIntent(intentFormMsgstr);
   return;
}

void VoiceRecognite::parseIntent(std::string &data)
{
    Json::Reader reader;
    Json::Value intentRoot;
    std::cout<<"parse"<<std::endl;

    if(!reader.parse(data,intentRoot,false)){
         std::cout<<"Intent Rooot parse error!!!"<<std::endl;
         return;
    }

    std::string intent = intentRoot["intent"].asCString();

    if(intent == "takedog"){
    std::cout << "intent was" << intent << std::endl;
    }

    /**
    * 获取物体名称
    */
    Json::Value slotsRoot = intentRoot["slots"];
    std::string objectStr = (slotsRoot[0])["normValue"].asString();

    std::cout << "正在抓取: " << objectStr << std::endl;
    data = objectStr;

    send(QString::fromStdString(objectStr));
    return;
}

int VoiceRecognite::startVoiceRecognition()
{
    hirop_msgs::StartListen startListen;
    clientSatrtListener = nVoice.serviceClient<hirop_msgs::StartListen>("start_listen");
    clientStopListener = nVoice.serviceClient<hirop_msgs::StopListen>("stop_listen");
    subUserIntent = nVoice.subscribe("/user_intent",1,&VoiceRecognite::listenVoice_callback,this);
    clientSatrtListener.call(startListen);
    if(startListen.response.reuslt == 0)
    {
        std::cout<<"start listen succeeful!!!"<<std::endl;
    }
    else
    {
        std::cout<<"start listent faial!!!"<<std::endl;
    }
    return 0;
}

int VoiceRecognite::stopVoiceRecognition()
{
    hirop_msgs::StopListen stopListen;
    clientStopListener.call(stopListen);
    if(stopListen.response.reuslt == 0)
    {
        std::cout<<"stop listen succeeful!!!"<<std::endl;
    }
    else
    {
        std::cout<<"stop listent faial!!!"<<std::endl;
    }
    return 0;
}

int VoiceRecognite::textToSoundPlay(QString text, QString fileName)
{
    int ret = -1;
    msc->TtsLogin();
    std::string Stext = text.toStdString();
    std::string file = fileName.toStdString();
    ret = msc->textToSpeech(file,Stext);
    msc->TtsLogout();
    std::cout<<"正在播放"<<std::endl;
    QSound::play(fileName);
    return 0;
}


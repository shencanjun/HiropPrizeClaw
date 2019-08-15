#include "voicerecognite.h"

VoiceRecognite::VoiceRecognite(ros::NodeHandle n)
{
    nVoice = n;
    clientSatrtListener = nVoice.serviceClient<hirop_msgs::StartListen>("start_listen");
    clientStopListener = nVoice.serviceClient<hirop_msgs::StopListen>("stop_listen");
}

VoiceRecognite::~VoiceRecognite()
{

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

   parseIntent(intentFormMsgstr);
   return;
}

void VoiceRecognite::parseIntent(std::string &data)
{
    Json::Reader reader;
    Json::Value intentRoot;

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

void VoiceRecognite::threadIntent()
{
    //boost::function0< void> voiceFun = boost::bind(&VoiceRecognite::listenVoice_callback,this);
    //thrdVoice = new boost::thread(voiceFun);

    //return;
}

int VoiceRecognite::startVoiceRecognition()
{
    hirop_msgs::StartListen startListen;
    if(clientSatrtListener.call(startListen))
    {
        std::cout<<"start listen succeeful!!!"<<std::endl;
        subUserIntent = nVoice.subscribe("/user_intent",1,&VoiceRecognite::listenVoice_callback,this);
        //threadIntent();
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
    if(clientStopListener.call(stopListen))
    {
        std::cout<<"stop listen succeeful!!!"<<std::endl;
    }
    else
    {
        std::cout<<"stop listent faial!!!"<<std::endl;
    }
    return 0;
}



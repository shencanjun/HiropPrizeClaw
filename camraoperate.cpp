#include "camraoperate.h"

CamraOperate::CamraOperate(ros::NodeHandle n)
{
    n_camra = n;
    clientCamra = n_camra.serviceClient<hkcamera_bridge::HkCameraData>("HkCameraData");
}

CamraOperate::~CamraOperate()
{

}

bool CamraOperate::connectCamra()
{
    hkCamraSrv.request.cmd = 1;
    if(clientCamra.call(hkCamraSrv))
    {

    }
    return true;
}

bool CamraOperate::disconnectCamra()
{
    hkCamraSrv.request.cmd = 4;
    if(clientCamra.call(hkCamraSrv))
    {

    }
    return true;
}

bool CamraOperate::takePicture()
{
    hkCamraSrv.request.cmd = 2;
    if(clientCamra.call(hkCamraSrv))
    {

    }
    return true;
}

bool CamraOperate::getImage()
{
    hkCamraSrv.request.cmd = 3;
    if(clientCamra.call(hkCamraSrv))
    {

    }
    return true;
}

void CamraOperate::getImage_callback(sensor_msgs::Image::ConstPtr &msg)
{
    sensor_msgs::Image img;
}

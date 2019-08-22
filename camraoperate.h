#ifndef CAMRAOPERATE_H
#define CAMRAOPERATE_H

#include <ros/ros.h>
#include <hkcamera_bridge/HkCameraData.h>
#include <sensor_msgs/Image.h>

class CamraOperate
{
public:
    CamraOperate(ros::NodeHandle n);
    ~CamraOperate();

    bool connectCamra();

    bool takePicture();

    bool getImage();

    bool disconnectCamra();

    void getImage_callback(sensor_msgs::Image::ConstPtr &msg);

private:
    ros::NodeHandle n_camra;
    ros::ServiceClient clientCamra;
    hkcamera_bridge::HkCameraData hkCamraSrv;

};

#endif // CAMRAOPERATE_H

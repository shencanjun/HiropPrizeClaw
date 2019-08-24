#ifndef CAMRAOPERATE_H
#define CAMRAOPERATE_H

#include <QObject>
#include <ros/ros.h>
#include <QMetaType>
#include <hkcamera_bridge/HkCameraData.h>
#include <sensor_msgs/Image.h>
#include <vision_bridge/ObjectArray.h>
#include <vision_bridge/ObjectInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <Calibration.h>
#include <geometry_msgs/Pose.h>
#include <vision_bridge/detection.h>

class CamraOperate : public QObject
{
    Q_OBJECT

public:
    CamraOperate(ros::NodeHandle n);
    ~CamraOperate();

    void startCamraService();

    bool connectCamra();

    bool takePicture();

    bool getImage();

    bool sendtImage();

    bool disconnectCamra();

    bool startEyeCalirating();

    bool writeCalibrateData(float rx, float ry, int num);

    bool readEyeData(float &cx, float &cy,float &rx, float &ry, int num);

    bool getCalibrateResult(double accu);

    bool readClibrateData();

    bool getDetesionResult(int x, int y, std::vector<double> &pose);

    bool opencvDrawing(int x, int y, int wide, int high);

    bool detectionSrv();

private:
    void getImage_callback(const sensor_msgs::ImageConstPtr &msg);

    void getObjectArray_callback(const vision_bridge::ObjectArray::ConstPtr &msg);\

public:
    void send(geometry_msgs::Pose pose) const{
        emit emitResultCam(pose);
    }

signals:
    void emitResultCam(geometry_msgs::Pose) const;

public:
    std::string imgFileName;
    std::string camCalibXmlFileName;
    cv::Mat colorImg;

private:
    ros::NodeHandle n_camra;
    ros::Subscriber subImage;
    ros::Subscriber subPose;
    ros::ServiceClient clientCamra;
    ros::ServiceClient clientDetection;
    hkcamera_bridge::HkCameraData hkCamraSrv;
    vision_bridge::detection VBDetectionSrv;

    cv_bridge::CvImagePtr color_ptr;
    geometry_msgs::Pose resultPose;

    EyeCalib2D *Calib2D;


};

#endif // CAMRAOPERATE_H

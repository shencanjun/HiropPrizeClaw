#ifndef CAMRAOPERATE_H
#define CAMRAOPERATE_H

#include <QObject>
#include <ros/ros.h>
#include <QMetaType>
#include <QLabel>
#include <hkcamera_bridge/HkCameraData.h>
#include <sensor_msgs/Image.h>
#include <vision_bridge/ObjectArray.h>
#include <vision_bridge/ObjectInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <Calibration.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_bridge/detection.h>

class CamraOperate : public QObject
{
    Q_OBJECT

public:
    CamraOperate(ros::NodeHandle n);
    ~CamraOperate();

    void startCamraService();

    int connectCamra();

    bool takePicture();

    bool getImage();

    bool sendtImage();

    bool disconnectCamra();

    bool startEyeCalirating();

    bool writeCalibrateData(float rx, float ry, int num);

    bool readEyeData(float &cx, float &cy, int num);

    bool getCalibrateResult(double &accu);

    bool readClibrateData();

    bool getDetesionResult(int x, int y, std::vector<double> &pose);

    bool opencvDrawing(int, int, int, int, cv::Mat&, QString, cv::String,cv::Scalar);

    bool detectionSrv();

    bool getCalibrateImage();

    bool getCalibraHomeData();

private:
    void getImage_callback(const sensor_msgs::ImageConstPtr &msg);

    void getObjectArray_callback(const vision_bridge::ObjectArray::ConstPtr &msg);

public:
    void send(bool have,std::vector<int> num,double rcoRate) const{
        emit emitResultCam(have, num,rcoRate);
    }

    void sendImage(cv::Mat mat) const{
        emit emitImagesignal(mat);
    }

signals:
    void emitResultCam(bool, std::vector<int> , double) const;
    void emitImagesignal(cv::Mat) const;

public:
    std::string imgFileName;
    std::string camCalibXmlFileName;
    QString AdetecImgFIle;
    cv::Mat colorImg;
    cv::Mat calibrateImage;

    std::vector< geometry_msgs::Pose > vecBear;
    std::vector< geometry_msgs::Pose > vecRabbit;
    std::vector< geometry_msgs::Pose > vecGiraffe;
    std::vector< geometry_msgs::Pose > vecDolphin;
    std::vector< geometry_msgs::Pose > vecLeopard;

private:
    ros::NodeHandle n_camra;
    ros::Subscriber subImage;
    ros::Subscriber subPose;
    ros::ServiceClient clientCamra;
    ros::ServiceClient clientDetection;
    hkcamera_bridge::HkCameraData hkCamraSrv;
    vision_bridge::detection VBDetectionSrv;

    cv_bridge::CvImagePtr color_ptr;
    geometry_msgs::PoseStamped resultPose;
    std::vector<geometry_msgs::PoseStamped> resultPoses;

    EyeCalib2D *Calib2D;
};

#endif // CAMRAOPERATE_H

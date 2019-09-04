#include "camraoperate.h"

CamraOperate::CamraOperate(ros::NodeHandle n)
{
    n_camra = n;
    Calib2D = new EyeCalib2D();
    clientCamra = n_camra.serviceClient<hkcamera_bridge::HkCameraData>("HkCameraData");
    clientDetection = n_camra.serviceClient<vision_bridge::detection>("detection");
}

CamraOperate::~CamraOperate()
{
    delete Calib2D;
}

void CamraOperate::startCamraService()
{
    subImage = n_camra.subscribe("camera/hk/image_raw", 1, &CamraOperate::getImage_callback, this);
    subPose = n_camra.subscribe("object_array", 1, &CamraOperate::getObjectArray_callback, this);
    std::cout<<"start Service"<<std::endl;
    return;
}

int CamraOperate::connectCamra()
{
    hkCamraSrv.request.cmd = 1;
    clientCamra.call(hkCamraSrv);
    std::cout<<hkCamraSrv.response.result<<std::endl;
    return hkCamraSrv.response.result;
}

bool CamraOperate::disconnectCamra()
{
    hkCamraSrv.request.cmd = 4;
    clientCamra.call(hkCamraSrv);
    if(hkCamraSrv.response.result != 0)
    {
        std::cout<<hkCamraSrv.response.result<<std::endl;
        return false;
    }
    return true;
}

bool CamraOperate::takePicture()
{
    hkCamraSrv.request.cmd = 2;
    clientCamra.call(hkCamraSrv);
    if(hkCamraSrv.response.result != 0)
    {
        return false;
    }
    return true;
}

bool CamraOperate::getImage()
{
    hkCamraSrv.request.cmd = 3;
    clientCamra.call(hkCamraSrv);
    if(hkCamraSrv.response.result != 0)
    {
        return false;
    }
    return true;
}

bool CamraOperate::detectionSrv()
{
    VBDetectionSrv.request.objectName = "CambriconYoloDetector";
    VBDetectionSrv.request.detectorName = "CambriconYoloDetector";
    VBDetectionSrv.request.detectorType = 1;
    VBDetectionSrv.request.detectorConfig = "";
    clientDetection.call(VBDetectionSrv);
    if(VBDetectionSrv.response.result != 0)
    {
        return false;
    }
    return true;
}

bool CamraOperate::getCalibrateImage()
{
    Calib2D->getEyeCalibImage(calibrateImage);
    sendImage(calibrateImage);
    return true;
}

bool CamraOperate::sendtImage()
{
    if(colorImg.empty()){
        ROS_ERROR("无图像");
        return false;
    }

    int ret = Calib2D->SetCalibImage(colorImg);

    return ret == 0 ? true : false;
}

void CamraOperate::getImage_callback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("GET IMAGE");
    colorImg.release();
    try
    {
        color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        colorImg = color_ptr->image;
        cv::imwrite(imgFileName,colorImg);
        sendImage(colorImg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void CamraOperate::getObjectArray_callback(const vision_bridge::ObjectArray::ConstPtr &msg)
{
    std::cout<<"getObjectArray"<<std::endl;
    cv::Mat ImgMat;
    geometry_msgs::Pose msgPose;
    bool have;
    double rcoRate = 0;
    vecBear.clear();
    vecRabbit.clear();
    vecGiraffe.clear();
    ImgMat = colorImg;
    if(msg->objects.size() <= 0){
        return;
    }
    std::cout<<"msg->objects.size():"<<msg->objects.size()<<std::endl;
    for(int i=0;i<msg->objects.size();i++){
        msgPose = msg->objects[i].pose.pose;

        if(!static_cast<bool >(msgPose.position.x)){    
            have = false;
            break;
        }

        if((static_cast<int >(msgPose.position.z)) == 1){
            vecBear.push_back(msgPose);
            opencvDrawing(msgPose.orientation.x,msgPose.orientation.y,
                          msgPose.orientation.z,msgPose.orientation.w,
                          ImgMat,AdetecImgFIle,cv::String("Bear"),
                          cv::Scalar(255,0,0));
        }
        else if((static_cast<int >(msgPose.position.z)) == 2){
            vecRabbit.push_back(msgPose);
            opencvDrawing(msgPose.orientation.x,msgPose.orientation.y,
                          msgPose.orientation.z,msgPose.orientation.w,
                          ImgMat,AdetecImgFIle,cv::String("Rabbit"),
                          cv::Scalar(0,255,0));
        }
        else if((static_cast<int >(msgPose.position.z)) == 3){
            vecGiraffe.push_back(msgPose);
            opencvDrawing(msgPose.orientation.x,msgPose.orientation.y,
                          msgPose.orientation.z,msgPose.orientation.w,
                          ImgMat,AdetecImgFIle,cv::String("Giraffe"),
                          cv::Scalar(0,0,255));
        }
        else{
            ;
        }
        have = true;
        rcoRate = msgPose.position.y;

//        std::cout<<"msgPose.pose.x:"<<msgPose.position.x<<std::endl;
//        std::cout<<"msgPose.pose.y:"<<msgPose.position.y<<std::endl;
//        std::cout<<"msgPose.pose.z:"<<msgPose.position.z<<std::endl;
//        std::cout<<"msgPose.orientation.x:"<<msgPose.orientation.x<<std::endl;
//        std::cout<<"msgPose.orientation.y:"<<msgPose.orientation.y<<std::endl;
//        std::cout<<"msgPose.orientation.z:"<<msgPose.orientation.z<<std::endl;
//        std::cout<<"msgPose.orientation.w:"<<msgPose.orientation.w<<std::endl;

    }
    sendImage(ImgMat);
    send(have,vecBear.size(),vecRabbit.size(),vecGiraffe.size(), rcoRate);
    ImgMat.release();
    return;
}

bool CamraOperate::startEyeCalirating()
{
    sendtImage();

    return true;
}

bool CamraOperate::readEyeData(float &cx, float &cy, int num)
{
    int ret = -1;
    std::vector<float> camVec;
    ret = Calib2D->readEyeCalibImage(camVec, num);
    if(camVec.size() < 2)
        return false;
    cx = camVec[0];
    cy = camVec[1];
    return ret == 0 ? true : false;
}

bool CamraOperate::writeCalibrateData(float rx, float ry, int num)
{
    int ret = -1;
    ret = Calib2D->writeEyeCalib(rx, ry, num);
    return ret == 0 ? true : false;
}

bool CamraOperate::getCalibrateResult(double &accu)
{
    int ret = -1;
    std::cout<<"camCalibXmlFileName:"<<camCalibXmlFileName<<std::endl;
    ret = Calib2D->estimaHom2D(camCalibXmlFileName,accu);
    return ret == 0 ? true : false;
}

bool CamraOperate::readClibrateData()
{
    int ret = -1;
    ret = Calib2D->readEyeCalibHomParam(camCalibXmlFileName);
    return ret == 0 ? true : false;
}

bool CamraOperate::getDetesionResult(int x, int y, std::vector<double> &pose)
{
    int ret = -2;
    ret = Calib2D->getAffineResult(x, y, pose);
    std::cout<<"getDetesionResult,ret:"<<ret<<std::endl;
    return ret == 0 ? true : false;
}

bool CamraOperate::opencvDrawing(int ax, int ay, int bx, int by,
                                 cv::Mat &ImageMat,QString imgFile,
                                 cv::String objName,cv::Scalar sca)
{
    try{
        cv::Rect rect(cv::Point(ax,ay), cv::Point(bx,by));
        cv::rectangle(ImageMat,rect,sca,3,cv::LINE_8,0);
        cv::putText(ImageMat, objName,cv::Point(ax,ay),cv::FONT_HERSHEY_SIMPLEX ,2,sca, 2, 8);
        cv::imwrite(imgFile.toStdString(),ImageMat);
    }catch(cv::Exception &e){
        ROS_ERROR("opencv exception: %s", e.what());
        return false;
    }
    return true;
}

bool CamraOperate::getCalibraHomeData()
{
    int ret  = -1;
    ret = Calib2D->readEyeCalibHomParam(camCalibXmlFileName);
    return ret == 0 ? true : false;
}

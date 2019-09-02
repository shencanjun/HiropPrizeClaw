#include "imagedialog.h"
#include "ui_imagedialog.h"

ImageDialog::ImageDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ImageDialog)
{
    ui->setupUi(this);
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(camOper, &CamraOperate::emitImagesignal, this, &ImageDialog::reviceImg);
    //showImageLabel();
}

ImageDialog::~ImageDialog()
{
    delete ui;
}

void ImageDialog::initCamObj(CamraOperate *cam)
{
    camOper = cam;
    return;
}

void ImageDialog::showImageLabel()
{
   // std::string path = "/home/ros/2.png";

//    cv::Mat mat1 = cv::imread(path);
//    //ui->label->clear();
//    LabelDisplayMat(ui->label,mat1);

    cv::Mat mat1 = cv::imread("/home/ros/2.png");


    return;
}

void ImageDialog::reviceImg(cv::Mat mat)
{
    std::cout<<"ni hao ni hao"<<std::endl;
    cv::imwrite("/home/ros/3.png",mat);
    QPixmap *pixmap = new QPixmap("/home/ros/3.png");
    pixmap->scaled(ui->label->size(), Qt::KeepAspectRatio);
    ui->label->setScaledContents(true);
    ui->label->setPixmap(*pixmap);
    return;
}

void ImageDialog::LabelDisplayMat(QLabel *label, cv::Mat mat)
{
    cv::Mat Rgb;
    QImage Img;
    if (mat.channels() == 3)//RGB Img
    {
        cv::cvtColor(mat, Rgb, CV_BGR2RGB);//颜色空间转换
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGB888);
    }
    else//Gray Img
    {
        Img = QImage((const uchar*)(mat.data), mat.cols, mat.rows, mat.cols*mat.channels(), QImage::Format_Indexed8);
    }

    label->setScaledContents(true);
    QSize qs = label->rect().size();

    QPixmap qimp = QPixmap::fromImage(Img);
    QPixmap nimp = qimp.scaled(qs);

    label->setPixmap(nimp);

    return;
}

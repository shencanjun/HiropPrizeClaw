#include "imagedialog.h"
#include "ui_imagedialog.h"

ImageDialog::ImageDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ImageDialog)
{
    ui->setupUi(this);
    qRegisterMetaType<cv::Mat>("cv::Mat");
    ui->label->installEventFilter(this);
    //showImageLabel();
}

ImageDialog::~ImageDialog()
{
    delete camOper;
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

    //cv::Mat mat1 = cv::imread("/home/ros/2.png");


    return;
}

void ImageDialog::reviceImg(cv::Mat mat)
{
    std::cout<<"ni hao ni hao"<<std::endl;
    cv::imwrite("./imgDialog.jpg",mat);
    QPixmap *pixmap = new QPixmap("./imgDialog.jpg");
    pixmap->scaled(ui->label->size(), Qt::KeepAspectRatio);
    ui->label->setScaledContents(true);
    ui->label->setPixmap(*pixmap);
    mat.release();
    delete pixmap;
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

    mat.release();
    Rgb.release();

    return;
}


bool ImageDialog::eventFilter(QObject  *obj, QEvent *event)
{
    if (obj == ui->label)//当事件发生在u1（为Qlabel型）控件上
    {
        if (event->type() == QEvent::MouseButtonDblClick)//当为双击事件时
        {
            sendclose();
            std::cout<<"ni hao ni hao"<<std::endl;
//            clikedcount++;
//            if (clikedcount % 2 == 0) //此处为双击一次全屏，再双击一次退出
//            {
//                //ui->label_show_image->setWindowFlags(Qt::Dialog);
//                //ui->label_show_image->showMaximized();//全屏显示

//            }
//            else
//            {
//                //imgDialog->close();
//                //ui->label_show_image->setWindowFlags(Qt::SubWindow);
//                //ui->label_show_image->showNormal();//退出全屏
//            };

        }
        return QObject::eventFilter(obj, event);
    }
}

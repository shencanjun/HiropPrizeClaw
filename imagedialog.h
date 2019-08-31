#ifndef IMAGEDIALOG_H
#define IMAGEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <opencv2/opencv.hpp>
#include "camraoperate.h"

namespace Ui {
class ImageDialog;
}

class ImageDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImageDialog(QWidget *parent = 0);
    ~ImageDialog();

    void initCamObj(CamraOperate *cam);

    void reviceImg(cv::Mat mat);

    void showImageLabel();

    void LabelDisplayMat(QLabel *label, cv::Mat mat);

private:
    Ui::ImageDialog *ui;
    CamraOperate *camOper;
};

#endif // IMAGEDIALOG_H

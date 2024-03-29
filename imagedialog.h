#ifndef IMAGEDIALOG_H
#define IMAGEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QMouseEvent>
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

    void showQuestion(QString);

public:
    void sendclose() const{
        emit emitCloseSignal();
    }

signals:
    void emitCloseSignal() const;

private:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    Ui::ImageDialog *ui;
    CamraOperate *camOper;
};

#endif // IMAGEDIALOG_H

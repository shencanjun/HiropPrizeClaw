#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#include <QDialog>
#include <QMessageBox>
#include <iostream>
#include "hsc3robot.h"

namespace Ui {
class CalibrateDialog;
}

class CalibrateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrateDialog(QWidget *parent = 0);
    ~CalibrateDialog();

private:
    Ui::CalibrateDialog *ui;
    HSC3ROBOT *Chsc3;
    LocData hscLocData;
    int raw;
    int column;
    const int cColumnX = 0;
    const int cColumnY = 1;
    const int rColumnX = 3;
    const int rColumnY = 4;

private:
    void RecordCalibrateData();

    void setCalibrateConfig();

    void getCalibrationResult();

    void SaveCalibration();

public:
    void setHsc3Object(HSC3ROBOT *MWhsc3);

};

#endif // CALIBRATEDIALOG_H

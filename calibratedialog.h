#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#include <QDialog>
#include <QMessageBox>
#include <QMenu>
#include <QAction>
#include <iostream>
#include <vector>
#include "hsc3robot.h"
#include "camraoperate.h"
#include "parseconfig.h"

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
    QMenu *tableMenu;
    QAction *action;
    HSC3ROBOT *Chsc3;
    CamraOperate *camCalib;
    LocData hscLocData;
    ParseConfig *parse;

    int raw;
    int column;
    static const int cColumnX = 0;
    static const int cColumnY = 1;
    static const int rColumnX = 3;
    static const int rColumnY = 4;

private:
    void startCalibration();

    void recordCalibrateData();

    void setCalibrateConfig();

    void getCalibrationResult();

    void SaveCalibration();

    void slotActionDelete();

    void on_tableWidget_customContextMenuRequested(const QPoint &pos);

public:
    void setOpeaObject(HSC3ROBOT *MWhsc3, CamraOperate *cam);
    void readCalibrateData();

    QString calibDataFileName;

};

#endif // CALIBRATEDIALOG_H

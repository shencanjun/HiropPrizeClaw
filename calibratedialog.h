#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#include <QDialog>

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
};

#endif // CALIBRATEDIALOG_H

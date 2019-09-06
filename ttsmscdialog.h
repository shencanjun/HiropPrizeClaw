#ifndef TTSMSCDIALOG_H
#define TTSMSCDIALOG_H

#include <QDialog>
#include <ttsmsc.h>
#include <qdebug.h>
#include <QSound>
#include <QSoundEffect>
#include <QFileDialog>
#include "voicerecognite.h"

namespace Ui {
class TtsMscDialog;
}

class TtsMscDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TtsMscDialog(QWidget *parent = 0);
    ~TtsMscDialog();

private:
    void textTtsMscBnt();

    void SourcePlay();

private:
    Ui::TtsMscDialog *ui;
    QString ttsFileName;
    Ttsmsc *msc;
};

#endif // TTSMSCDIALOG_H

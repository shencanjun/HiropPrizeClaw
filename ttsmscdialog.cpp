#include "ttsmscdialog.h"
#include "ui_ttsmscdialog.h"

TtsMscDialog::TtsMscDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TtsMscDialog)
{
    ui->setupUi(this);
    msc = new Ttsmsc();

    connect(ui->pushButton_tts,&QPushButton::clicked,this,&TtsMscDialog::textTtsMscBnt);
    connect(ui->pushButton_souPlay,&QPushButton::clicked,this,&TtsMscDialog::SourcePlay);

    ui->textBrowser_context->setReadOnly(false);

    /* 默认wav音频头部数据 */
    wave_pcm_hdr default_wav_hdr =
    {
            { 'R', 'I', 'F', 'F' },
            0,
            {'W', 'A', 'V', 'E'},
            {'f', 'm', 't', ' '},
            16,
            1,
            1,
            16000,
            32000,
            2,
            16,
            {'d', 'a', 't', 'a'},
            0,
    };
}

TtsMscDialog::~TtsMscDialog()
{
    //delete msc;
    delete ui;
}

void TtsMscDialog::textTtsMscBnt()
{
    ttsFileName = QFileDialog::getSaveFileName(this,"打开图片",".","*.wav *.pcm");
    if(ttsFileName.isEmpty())
        return;
    std::cout<<msc->TtsLogin()<<std::endl;
    QString textCur = ui->textBrowser_context->toPlainText();
    qDebug()<<"textCur"<<textCur;
    std::string text = textCur.toStdString();
    std::string fileName = ttsFileName.toStdString();
    std::cout<<msc->textToSpeech(fileName,text)<<std::endl;
    msc->TtsLogout();
    std::cout<<"return"<<std::endl;
    return;
}

void TtsMscDialog::SourcePlay()
{
    send(ttsFileName);
    std::cout<<"return"<<std::endl;
    return;
}

void TtsMscDialog::play()
{
    QSound::play(ttsFileName);
    return;
}


qint64 TtsMscDialog::getAudioTime(const QString &filePath)
{
    QFile file(filePath);
    if (file.open(QIODevice::ReadOnly)) {
        qint64 fileSize = file.size();
        qint64 time = fileSize / (16000.0 * 2.0);
        file.close();
        return time;
    }
    return -1;
}

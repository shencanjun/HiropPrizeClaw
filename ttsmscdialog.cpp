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
//    QSoundEffect effect;
//    qDebug()<<"ttsFileName : "<< ttsFileName;
//    effect.setSource(QUrl::fromLocalFile("sound.wav"));
//    //循环播放
//    //effect.setLoopCount(QSoundEffect::Infinite);
//    //设置音量，0-1
//    effect.setVolume(0.25f);
//    effect.play();
    //std::cout<<"effect.isPlaying()"<<effect.isPlaying()<<std::endl;
//    while(effect.isPlaying())
//    {
//        usleep(500000);
//    }
//    return;

    QSound sound(ttsFileName);
    std::cout<<"正在播放"<<std::endl;
    sound.play(ttsFileName);
    std::cout<<"return"<<std::endl;
//    while(sound.isFinished()){
//        sleep(1.0);
//    }
//    std::cout<<"sleep5 sound.isFinished = "<<sound.isFinished()<<std::endl;
    return;
}

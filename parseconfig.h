#ifndef PARSECONFIG_H
#define PARSECONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFile>
#include <QtDebug>
#include <QIODevice>
#include <QTime>
#include <QtGlobal>

#define CALDATA_ELEMENT "CalibrateData"
#define ACC_ELEMENT "Accuracy"
#define COMX_ELEMENT "compensationX"
#define COMY_ELEMENT "compensationY"
#define VERSION_ATTRIBUTE "Version"
#define CONFIG "config"
#define QUESTIONS "questions"
#define QUESTION "question"
#define ANSWER "answer"

class ParseConfig
{
public:
    ParseConfig();
    ~ParseConfig();

    bool writeXmlFile(QString fileName,double acc, double compensationX,double compensationY);

    bool readCalibXML(QString fileName,double &acc, double &comx,double &comy);

    bool readMainXml(QString fileName,QString StartElementStr, QString ElementStr,QString &conStr);

    void readQuestions(QString fileName, int &queNum);

    void resultQuestion(QString &qestion, QString &anwser);

    QString errorString() const;  // 错误信息

private:
    void readXBEL();   // 读取根元素 <CalibrateData>
    void readAcc();    // 读取元素 <Accuracy>
    void readComX();   // 读取元素 <compensationX>
    void readComY();   // 读取元素 <compensationY>
    void get_random_number(int &index);

    QXmlStreamReader xmlReader;
    QXmlStreamReader xmlMReader;
    QXmlStreamReader xmlQesReader;
    QXmlStreamWriter xmlWriter;

    QStringList qesList;
    QStringList anwserList;

    double acc_;
    double comx_;
    double comy_;

    int quesNum;
};

#endif // PARSECONFIG_H

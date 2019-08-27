#include "parseconfig.h"

ParseConfig::ParseConfig()
{

}

ParseConfig::~ParseConfig()
{

}

bool ParseConfig::writeXmlFile(QString fileName,double acc, double compensationX,double compensationY)
{
    QString strFile(fileName);

    QFile file(strFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) { // 只写模式打开文件
        qDebug() << QString("Cannot write file %1(%2).").arg(strFile).arg(file.errorString());
        return false;
    }

    QXmlStreamWriter writer(&file);
    // writer.setCodec("GBK");  // XML 编码
    writer.setAutoFormatting(true); // 自动格式化
    writer.writeStartDocument("1.0", true);  // 开始文档（XML 声明）
    writer.writeComment(QString::fromLocal8Bit("标定精度"));  // 注释
    //    writer.writeProcessingInstruction("xml-stylesheet type=\"text/css\" href=\"style.css\"");  // 处理指令

    writer.writeStartElement("CalibrateData");  // 开始根元素 <Calibrate>

    writer.writeTextElement(QString::fromLocal8Bit("Accuracy"), QString::number(acc,'f',4));
    writer.writeTextElement(QString::fromLocal8Bit("compensationX"), QString::number(compensationX,'f',4));
    writer.writeTextElement(QString::fromLocal8Bit("compensationY"), QString::number(compensationY,'f',4));

    writer.writeEndElement();  // 结束根元素 </Blogs>
    writer.writeEndDocument();  // 结束文档

    file.close();  // 关闭文件
}

bool ParseConfig::readMainXml(QString fileName,QString StartElementStr, QString ElementStr,QString &conStr)
{
    QFile *file = new QFile(fileName);
    if (!file->open(QFile::ReadOnly | QFile::Text)) { // 只写模式打开文件
        std::cout<<"open xml file faile!!!"<<std::endl;
        return false;
    }
    std::cout<<"fileName ."<<fileName.toStdString()<<std::endl;
    xmlMReader.setDevice(file);
    std::cout<<"-------------------"<<std::endl;
    if(xmlMReader.readNextStartElement()){
        std::cout<<"11111111111111 ."<<std::endl;
        QString strName = xmlMReader.name().toString();
        if(strName == CONFIG)
        {
            std::cout<<"22222222222."<<std::endl;
            Q_ASSERT(xmlMReader.isStartElement() && xmlMReader.name().toString() == CONFIG);

            if(xmlMReader.readNextStartElement()){
                //Q_ASSERT(xmlMReader.isStartElement() && xmlMReader.name().toString() == StartElementStr);
                QString staName = xmlMReader.name().toString();
                std::cout<<"staStr:"<<staName.toStdString()<<std::endl;
                if(staName == StartElementStr){

                    while (xmlMReader.readNextStartElement()) {
                        if (xmlMReader.name().toString() == ElementStr){
                            std::cout<<"33333333333333 ."<<std::endl;
                            Q_ASSERT(xmlMReader.isStartElement() && xmlMReader.name().toString() == ElementStr);

                            conStr = xmlMReader.readElementText();
                            qDebug() << QString::fromLocal8Bit("conStr：%1").arg(conStr);
                        }
                        else
                        xmlMReader.skipCurrentElement();  // 跳过当前元素
                   }
                }
            }
        }
        else {
            std::cout<<"XML file format error 11 ."<<std::endl;
            xmlMReader.raiseError("XML file format error.");
        }
    }
    else {
        std::cout<<"XML file format error."<<std::endl;
        xmlMReader.raiseError("XML file format error 22 .");
    }
    return true;
}

bool ParseConfig::readCalibXML(QString fileName, double &acc, double &comx, double &comy)
{
    QFile *file = new QFile(fileName);
    if (!file->open(QFile::ReadOnly | QFile::Text)) { // 只写模式打开文件
        std::cout<<"file false"<<std::endl;
        return false;
    }
    std::cout<<"file true"<<std::endl;
    xmlReader.setDevice(file);
    if (xmlReader.readNextStartElement()) {
        QString strName = xmlReader.name().toString();
        if (strName== CALDATA_ELEMENT) {  // 获取根元素
            readXBEL();
        } else {
            xmlReader.raiseError("XML file format error.");
        }
    }
    acc = acc_;
    comx = comx_;
    comy = comy_;

    return !xmlReader.error();
}

QString ParseConfig::errorString() const
{
    return QString("Error:%1  Line:%2  Column:%3")
        .arg(xmlReader.errorString())
        .arg(xmlReader.lineNumber())
        .arg(xmlReader.columnNumber());
}

void ParseConfig::readXBEL()
{
    Q_ASSERT(xmlReader.isStartElement() && xmlReader.name().toString() == CALDATA_ELEMENT);

    while (xmlReader.readNextStartElement()) {
        if (xmlReader.name().toString() == ACC_ELEMENT)
            readAcc();
        else if (xmlReader.name().toString() == COMX_ELEMENT)
            readComX();
        else if (xmlReader.name().toString() == COMY_ELEMENT)
            readComY();
        else
            xmlReader.skipCurrentElement();  // 跳过当前元素
    }
}

void ParseConfig::readAcc()
{
    Q_ASSERT(xmlReader.isStartElement() && xmlReader.name().toString() == ACC_ELEMENT);

    QString strAuthor = xmlReader.readElementText();
    acc_ = strAuthor.toDouble();
    qDebug() << QString::fromLocal8Bit("ACC：%1").arg(strAuthor);
    return;
}

void ParseConfig::readComX()
{
    Q_ASSERT(xmlReader.isStartElement() && xmlReader.name().toString() == COMX_ELEMENT);

    QString strAuthor = xmlReader.readElementText();
    comx_ = strAuthor.toDouble();
    qDebug() << QString::fromLocal8Bit("COMX：%1").arg(strAuthor);
    return;
}

void ParseConfig::readComY()
{
    Q_ASSERT(xmlReader.isStartElement() && xmlReader.name().toString() == COMY_ELEMENT);

    QString strAuthor = xmlReader.readElementText();
    comy_ = strAuthor.toDouble();
    qDebug() << QString::fromLocal8Bit("COMY：%1").arg(strAuthor);
    return;
}

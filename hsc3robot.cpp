#include "hsc3robot.h"

HSC3ROBOT::HSC3ROBOT()
{
    comapi = new CommApi();
    //关闭自动重连功能，否则连接失败
    comapi->setAutoConn(false);

    proMotion = new ProxyMotion(comapi);
    proSys = new ProxySys(comapi);
    proVar = new ProxyVar(comapi);
    proVm = new ProxyVm(comapi);
    proIo = new ProxyIO(comapi);
}

HSC3ROBOT::~HSC3ROBOT()
{
    delete proIo;
    delete proVm;
    delete proVar;
    delete proSys;
    delete proMotion;
    delete comapi;
}

bool HSC3ROBOT::connectIPC(std::string IPstr, uint16_t port)
{
    std::cout<<"IPstr = " << IPstr
             <<",port = " << port <<std::endl;
    ret = comapi->connect(IPstr, port);
    std::cout<<"ret = " << ret <<std::endl;
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::disconnectIPC()
{
    ret = comapi->disconnect();
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::isConnectIPC()
{
    return comapi->isConnected();
}

bool HSC3ROBOT::setHscEnanle(bool en)
{
    ret = proMotion->setGpEn(gpId,en);
    std::cout<<"ret = "<<ret << std::endl;
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscEnanle(bool &en)
{
    ret = proMotion->getGpEn(gpId,en);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscLoadPRG(std::string progname,std::string path)
{
    ret = proVm->load(path,progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscUnloadPRG(std::string progname)
{
    ret = proVm->unload(progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscPrgStart(std::string progname)
{    
    ret = proVm->start(progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscPrgStop(std::string progname)
{
    ret = proVm->stop(progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscClearFault()
{
    ret = proSys->reset();
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscR(int index, double value)
{
    ret = proVar->setR(index,value);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscR(int index, double &value)
{
    ret = proVar->getR(index,value);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscLR(int index, LocData pos)
{
    LocPos locpos;
    locpos.ufNum = -1;
    locpos.utNum = -1;
    proMotion->getConfig(gpId,locpos.config);
    locpos.vecPos = pos;
    ret = proVar->setLR(gpId,index,locpos);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscLR(int index, LocData& pos)
{
    LocPos locpos;
    ret = proVar->getLR(gpId,index,locpos);
    pos = locpos.vecPos;
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscLoc(LocData &posData)
{
    ret = proMotion->getLocData(gpId,posData);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getFaultMessage(ErrLevel &level, std::string &msg)
{
    uint64_t code;
    uint32_t time = 100;
    ret = proSys->getMessage(level,code,msg,time);
    std::cout<<"msg ret ="<<ret <<std::endl;
    return (ret == 0 || ret == KM_ERR_NO_MESSAGE) ? true : false;
}

bool HSC3ROBOT::getHscProInfo(const std::string &fileName, ProgInfo &info)
{
    ret = proVm ->getProgInfo(fileName, info);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscVord(int vord)
{
    //ret = proMotion->setJogVord();
    ret = proMotion->setVord(vord);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscIoValue(int index, bool &value)
{
    ret = proIo->getDout(index,value);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscIoValue(int index, bool value)
{
    ret = proIo->setDout(index, value);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscMode(OpMode mode)
{
    ret = proMotion->setOpMode(mode);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::sendProg(QString fileName)
{
    QString fullName = "sshpass -p 123456 scp ";
    QString hostname = " gm@10.10.56.214:/usr/codesys/hsc3_app/script/";
    fullName.append(fileName);
    fullName.append(hostname);

    qDebug()<<"fullName:"<<fullName;

    QByteArray bary = fullName.toLatin1();

    const char *cmd = bary.data();
    char* rester;
    if(!executeCMD(cmd, rester)){
        return false;
    }
    return true;
}

bool HSC3ROBOT::executeCMD(const char *cmd, char *result)
{
    char buf_ps[1024];
    char ps[1024]={0};
    FILE *ptr;
    strcpy(ps, cmd);
    if((ptr=popen(ps, "r"))!=NULL)
    {
        std::cout<<11111<<std::endl;
        while(fgets(buf_ps, 1024, ptr)!=NULL)
        {
            // 可以通过这行来获取shell命令行中的每一行的输出
            std::cout<<"buf_ps:"<<buf_ps<<std::endl;
            strcat(result, buf_ps);
            if(strlen(result)>1024)
                break;
        }
        pclose(ptr);
        ptr = NULL;
    }
    else
    {
        std::cout<<2222222<<std::endl;
        printf("popen %s error\n", ps);
        return false;
    }
    return true;
}

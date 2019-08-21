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
}

HSC3ROBOT::~HSC3ROBOT()
{
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
    uint32_t time = 500;
    ret = proSys->getMessage(level,code,msg,time);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscProInfo(const std::string &fileName, ProgInfo &info)
{
    ret = proVm ->getProgInfo(fileName, info);
    return ret == 0 ? true : false;
}




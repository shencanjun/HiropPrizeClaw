#include "hsc3robot.h"

HSC3ROBOT::HSC3ROBOT()
{
    comapi = new CommApi();
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
    ret = comapi->connect(IPstr,port);
    return ret==0 ? true : false;
}

bool HSC3ROBOT::disconnectIPC()
{
    ret = comapi->disconnect();
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::setHscEnanle(bool en)
{
    ret = proMotion->setGpEn(gpId,en);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscEnanle(bool &en)
{
    ret = proMotion->getGpEn(gpId,en);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscLoadPRG(std::string path = "./script",std::string progname)
{
    ret = proVm->load(path,progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscUnloadPRG(std::string progname)
{
    ret = proVm->unload(progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscPrgStart()
{
    std::string progname;
    ret = proVm->start(progname);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::HscPrgStop()
{
    std::string progname;
    ret = proVm->stop(progname);
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

bool HSC3ROBOT::setHscLR(int index, LocPos pos)
{
    ret = proVar->setLR(gpId,index,pos);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscLR(int index, LocPos& pos)
{
    ret = proVar->getLR(gpId,index,pos);
    return ret == 0 ? true : false;
}

bool HSC3ROBOT::getHscLoc(LocData &posData)
{
    ret = proMotion->getLocData(goId,posData);
    return ret == 0 ? true : false;
}




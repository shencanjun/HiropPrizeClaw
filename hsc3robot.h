#ifndef HSC3ROBOT_H
#define HSC3ROBOT_H

#include "3rdparty/include/CommApi.h"
#include "3rdparty/include/Hsc3Def.h"
#include "3rdparty/include/ErrDef.h"
#include "3rdparty/include/proxy/ProxyMotion.h"
#include "3rdparty/include/proxy/ProxyVar.h"
#include "3rdparty/include/proxy/ProxySys.h"
#include "3rdparty/include/proxy/ProxyVm.h"
#include <QException>

using namespace Hsc3::Comm;
using namespace Hsc3::Proxy;

class HSC3ROBOT
{
public:
    HSC3ROBOT();
    ~HSC3ROBOT();

public:
    bool connectIPC(std::string IPstr, uint16_t port);

    bool disconnectIPC();

    bool setHscEnanle(bool en);

    bool getHscEnanle(bool& en);

    bool HscLoadPRG(std::string progname,std::string path = "./script");

    bool HscUnloadPRG(std::string progname);

    bool HscPrgStart();

    bool HscPrgStop();

    bool setHscR(int index, double value);

    bool getHscR(int index, double& value);

    bool setHscLR(int index, LocData pos);

    bool getHscLR(int index, LocData& pos);

    bool getHscLoc(LocData& posData);

protected:

    CommApi *comapi;
    ProxyMotion *proMotion;
    ProxySys *proSys;
    ProxyVar *proVar;
    ProxyVm *proVm;
    HMCErrCode ret;

    LocPos getLRData;
    LocPos resultLRData;

    LocData LocPosData;

    const int8_t gpId = 0;
};

#endif // HSC3ROBOT_H

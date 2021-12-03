#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include <new>
#include <arpa/inet.h>
#include <sstream>


#include <sys/types.h>
#include <sys/stat.h>

#include <cantProceed.h>
#include <ellLib.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <drvSup.h>
#include <epicsExport.h>

#include <bsssAsynDriver.h>

#include <yamlLoader.h>

static const char *drverName = "bsssAsynDriver";

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *port;
    char            *reg_path;
    bsssAsynDriver  *pBsssDrv;
    ELLLIST         *pBsssList;
} pDrvList_t;


static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsssAsynDriver(init_drvList)");
        ellInit(pDrvEllList);
    }
    return;
}

static pDrvList_t *find_drvLast(void)
{
    init_drvList();

    if(ellCount(pDrvEllList)) return (pDrvList_t *) ellLast(pDrvEllList);
    else                      return (pDrvList_t *) NULL;

}

static int prep_drvAnonimous(void)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsssAsynDriver(prep_drvAnonimous)");

    p->named_root = NULL;
    p->port       = NULL;
    p->reg_path   = NULL;
    p->pBsssDrv   = NULL;
    p->pBsssList  = NULL;

    ellAdd(pDrvEllList, &p->node);
    return ellCount(pDrvEllList);
}



bsssAsynDriver::bsssAsynDriver(const char *portName, const char *reg_path, ELLLIST *pBsssEllList,  const char *named_root)
    : asynPortDriver(portName,
                         1,  /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                                         NUM_BSA_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
#endif /* asyn version check, under 4.32 */
                                         asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask |
                                         asynInt32ArrayMask | asynInt64ArrayMask | asynFloat64ArrayMask,    /* Interface mask */
                                         asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask | asynEnumMask |
                                         asynInt32ArrayMask | asynInt64ArrayMask | asynFloat64ArrayMask,       /* Interrupt mask */
                                         1,    /* asynFlags. This driver does block and it is non multi-device, so flag is 1. */
                                         1,    /* Auto connect */
                                         0,    /* Default priority */
                                         0)    /* Default stack size */

{
    if(!pBsssEllList || !ellCount(pBsssEllList)) return;   /* if there is no Bsss data channels in the list, nothing to do */
    this->pBsssEllList = pBsssEllList;

    Path root_ = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
    if(!root_) {
        printf("BSSS driver: could not find root path\n");
        return;
    }

    Path reg_ = root_->findByName(reg_path);
    if(!reg_) {
        printf("BSSS driver: could not find registers at path %s\n", reg_path);
        return;
    }
    this->pBsss = new Bsss::BsssYaml(reg_);  /* create API interface */

    SetupAsynParams();
}

bsssAsynDriver::~bsssAsynDriver()
{
}

void bsssAsynDriver::SetupAsynParams(void)
{
    char param_name[64];

//    sprintf(param_name, 
}

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

#include <bldStream.h>
#include <bsssAsynDriver.h>
#include <bsaAsynDriver.h>

#include <yamlLoader.h>

static const char *drverName = "bsssAsynDriver";

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *port_name;
    char            *reg_path;
    bsssAsynDriver  *pBsssDrv;
    ELLLIST         *pBsssEllList;
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

static pDrvList_t *find_drvByPort(const char *port_name)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->port_name && strlen(p->port_name) && !strcmp(p->port_name, port_name)) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}

static int prep_drvAnonimous(void)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsssAsynDriver(prep_drvAnonimous)");

    p->named_root = NULL;
    p->port_name  = NULL;
    p->reg_path   = NULL;
    p->pBsssDrv   = NULL;
    p->pBsssEllList  = NULL;

    ellAdd(pDrvEllList, &p->node);
    return ellCount(pDrvEllList);
}


static int bsssAdd(const char *bsssKey, bsssDataType_t type, double *slope, double *offset)
{
    pDrvList_t * pl = find_drvLast();

    if(pl){
        if(pl->pBsssDrv || (pl->port_name && strlen(pl->port_name))) pl = NULL;  /* the driver node has been configured,
                                                                                    need to make another one */
    }

    while(!pl) {
        prep_drvAnonimous();
        pl = find_drvLast();
    }

    while(!pl->pBsssEllList) {   /* initialize the linked list once */
        pl->pBsssEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsssAsynDriver (bsssAdd)");
        ellInit(pl->pBsssEllList);
    }

    bsssList_t *p = (bsssList_t *) mallocMustSucceed(sizeof(bsssList_t), "bsssAsynDriver (bsssAdd)");
    strcpy(p->bsss_name, bsssKey);
    for(int i = 0; i < NUM_BSSS_CHN; i++) {
        p->p_bsss[i] = -1;              /* initialize paramters with invalid */
        p->pname_bsss[i][0] = '\0';     /* initialize with a null string */
        p->pname_bsssPID[i][0] = '\0';  /* initialize with a null string */ 
    }

    p->type    = type;
    p->pslope  = slope;
    p->poffset = offset;

    ellAdd(pl->pBsssEllList, &p->node);
    return 0;
}


static int associateBsaChannels(const char *port_name)
{
    ELLLIST *pBsaEllList = find_bsaChannelList(port_name);

    if(!pBsaEllList) return -1;
    bsaList_t * p = (bsaList_t *) ellFirst(pBsaEllList);

    while(p) {
        bsssAdd(p->bsa_name, bsssDataType_t(p->type), &p->slope, &p->offset);
        p = (bsaList_t *) ellNext(&p->node);
    }
    printf("Associate %d of channels from bsa port(%s) \n", ellCount(find_drvLast()->pBsssEllList), port_name);

    return 0;
}


static void bsss_callback(void *pUsr, void *buf, unsigned size)
{
    ((bsssAsynDriver *)pUsr)->bsssCallback(buf, size);
}

bsssAsynDriver::bsssAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsssEllList,  const char *named_root)
    : asynPortDriver(portName,
                                        1,  /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                                         NUM_BSSS_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
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

    registerBsssCallback(named_root, bsss_callback, (void *) this);
}

bsssAsynDriver::~bsssAsynDriver()
{
}

void bsssAsynDriver::SetupAsynParams(void)
{
    char param_name[64];

    // BSSS Status Monitoring
    sprintf(param_name, CURRPACKETSIZE_STR);   createParam(param_name, asynParamInt32, &p_currPacketSize);
    sprintf(param_name, CURRPACKETSTATUS_STR); createParam(param_name, asynParamInt32, &p_currPacketStatus);
    sprintf(param_name, CURRPULSEIDL_STR);     createParam(param_name, asynParamInt32, &p_currPulseIdL);
    sprintf(param_name, CURRTIMESTAMPL_STR);   createParam(param_name, asynParamInt32, &p_currTimeStampL);
    sprintf(param_name, CURRDELTA_STR);        createParam(param_name, asynParamInt32, &p_currDelta);
    sprintf(param_name, PACKETCOUNT_STR);      createParam(param_name, asynParamInt32, &p_packetCount);
    sprintf(param_name, PAUSED_STR);           createParam(param_name, asynParamInt32, &p_paused);
    sprintf(param_name, DIAGNCLOCKRATE_STR);   createParam(param_name, asynParamInt32, &p_diagnClockRate);
    sprintf(param_name, DIAGNSTROBERATE_STR);  createParam(param_name, asynParamInt32, &p_diagnStrobeRate);
    sprintf(param_name, EVENTSEL0RATE_STR);    createParam(param_name, asynParamInt32, &p_eventSel0Rate);

    // BSSS Status Control
    sprintf(param_name, PACKETSIZE_STR);       createParam(param_name, asynParamInt32, &p_packetSize);
    sprintf(param_name, ENABLE_STR);           createParam(param_name, asynParamInt32, &p_enable);

    for(int i = 0; i < NUM_BSSS_DATA_MAX; i++) {
        sprintf(param_name, CHANNELMASK_STR, i); createParam(param_name, asynParamInt32, &p_channelMask[i]);
        sprintf(param_name, CHANNELSEVR_STR, i); createParam(param_name, asynParamInt32, &p_channelSevr[i]);
    }

    // BSSS Rate Controls
    for(int i = 0; i < NUM_BSSS_CHN; i++) {
        sprintf(param_name, EDEFENABLE_STR, i);createParam(param_name, asynParamInt32, &p_edefEnable[i]);
        sprintf(param_name, RATEMODE_STR, i);  createParam(param_name, asynParamInt32, &p_rateMode[i]);
        sprintf(param_name, FIXEDRATE_STR, i); createParam(param_name, asynParamInt32, &p_fixedRate[i]);
        sprintf(param_name, ACRATE_STR, i);    createParam(param_name, asynParamInt32, &p_acRate[i]);
        sprintf(param_name, TSLOTMASK_STR, i); createParam(param_name, asynParamInt32, &p_tSlotMask[i]);
        sprintf(param_name, EXPSEQNUM_STR, i); createParam(param_name, asynParamInt32, &p_expSeqNum[i]);
        sprintf(param_name, EXPSEQBIT_STR, i); createParam(param_name, asynParamInt32, &p_expSeqBit[i]);
        sprintf(param_name, DESTMODE_STR, i);  createParam(param_name, asynParamInt32, &p_destMode[i]);
        sprintf(param_name, DESTMASK_STR, i);  createParam(param_name, asynParamInt32, &p_destMask[i]);
        sprintf(param_name, RATELIMIT_STR, i); createParam(param_name, asynParamInt32, &p_rateLimit[i]);
    }

    // set up dyanamic paramters
    for(int i = 0; i < NUM_BSSS_CHN; i++) {
        bsssList_t *p  = (bsssList_t *) ellFirst(this->pBsssEllList);
        while(p) {
            sprintf(param_name, BSSSPV_STR,  p->bsss_name, i); createParam(param_name, asynParamFloat64, &p->p_bsss[i]);    strcpy(p->pname_bsss[i],    param_name);
            sprintf(param_name, BSSSPID_STR, p->bsss_name, i); createParam(param_name, asynParamInt64,   &p->p_bsssPID[i]); strcpy(p->pname_bsssPID[i], param_name);
            p = (bsssList_t *) ellNext(&p->node);
        }
    }

}


void bsssAsynDriver::SetRate(int chn)
{
    uint32_t rateMode, fixedRate, acRate, tSlotMask, expSeqNum, expSeqBit;

    getIntegerParam(p_rateMode[chn], (epicsInt32*) &rateMode);
    switch(rateMode) {
        case 0: /* fixed rate mode */
            getIntegerParam(p_fixedRate[chn], (epicsInt32*) &fixedRate);
            pBsss->setFixedRate(chn, fixedRate);
            break;
        case 1: /* AC rate mode */
            getIntegerParam(p_acRate[chn], (epicsInt32*) &acRate);
            getIntegerParam(p_tSlotMask[chn], (epicsInt32*) &tSlotMask);
            pBsss->setACRate(chn, tSlotMask, acRate);
            break;
        case 2: /* Seq rate mode */
            getIntegerParam(p_expSeqNum[chn], (epicsInt32*) &expSeqNum);
            getIntegerParam(p_expSeqBit[chn], (epicsInt32*) &expSeqBit);
            pBsss->setSeqRate(chn, expSeqNum, expSeqBit);
            break;
        default:  /* nothing todo */
            break;
    }
}

void bsssAsynDriver::SetDest(int chn)
{
    uint32_t destMode, destMask;

    getIntegerParam(p_destMode[chn], (epicsInt32*) &destMode);
    getIntegerParam(p_destMask[chn], (epicsInt32*) &destMask);

    switch(destMode) {
        case 2:  /* Inclusion */
            pBsss->setDestInclusion(chn, destMask);
            break;
        case 1:  /* Exclusion */
            pBsss->setDestExclusion(chn, destMask);
            break;
        case 0:  /* Disable */
            pBsss->setDestDisable(chn);
            break;
        default:  /* nothing to do */
            break;
    }
}

void bsssAsynDriver::MonitorStatus(void)
{
    uint32_t v;

    pBsss->getCurrPacketSize(&v);    setIntegerParam(p_currPacketSize, (epicsInt32) v);
    pBsss->getCurrPacketState(&v);   setIntegerParam(p_currPacketStatus, (epicsInt32) v);
    pBsss->getCurrPulseIdL(&v);      setIntegerParam(p_currPulseIdL, (epicsInt32) v);
    pBsss->getCurrTimeStampL(&v);    setIntegerParam(p_currTimeStampL, (epicsInt32) v);
    pBsss->getCurrDelta(&v);         setIntegerParam(p_currDelta, (epicsInt32) v);
    pBsss->getPacketCount(&v);       setIntegerParam(p_packetCount, (epicsInt32) v);
    pBsss->getPaused(&v);            setIntegerParam(p_paused, (epicsInt32) v);
    pBsss->getDiagnClockRate(&v);    setIntegerParam(p_diagnClockRate, (epicsInt32) v);
    pBsss->getDiagnStrobeRate(&v);   setIntegerParam(p_diagnStrobeRate, (epicsInt32) v);
    pBsss->getEventSel0Rate(&v);     setIntegerParam(p_eventSel0Rate, (epicsInt32) v);

    callParamCallbacks();
}

asynStatus bsssAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";

    /* set the parameter in the parameter library */
    status = (asynStatus) setIntegerParam(function, value);

    if(function == p_packetSize) {
        pBsss->setPacketSize((uint32_t) value);
        goto done;
    }
    else if(function == p_enable) {
        pBsss->enablePacket((uint32_t) value);
        goto done;
    }

    for(int i = 0; i < NUM_BSSS_DATA_MAX; i++) {
        if(function == p_channelMask[i]) {
            pBsss->setChannelMask(i, (uint32_t) value);
            goto done;
        }
        else if(function == p_channelSevr[i]) {
            pBsss->setChannelSevr(i, (uint64_t) value);
            goto done;
        }
    }

    for(int i = 0; i < NUM_BSSS_CHN; i++) {
        if(function == p_rateMode[i]  ||
           function == p_fixedRate[i] ||
           function == p_acRate[i]    ||
           function == p_tSlotMask[i] ||
           function == p_expSeqNum[i] ||
           function == p_expSeqBit[i]) {
            SetRate(i);
            goto done;
        }

        else if(function == p_destMode[i] ||
                function == p_destMask[i]) {
            SetDest(i);
            goto done;
        }
        else if(function == p_edefEnable[i]) {
            pBsss->setEdefEnable(i, (uint32_t) value);
            goto done;
        }
        else if(function == p_rateLimit[i]) {
            pBsss->setRateLimit(i, (uint32_t) value);
            goto done;
        }
    }

    done:
    callParamCallbacks();
    return status;
}


void bsssAsynDriver::bsssCallback(void *p, unsigned size)
{
     uint32_t *buf         = (uint32_t *) p;
     uint32_t *p_uint32    = buf + IDX_DATA;
     int32_t  *p_int32     = (int32_t *) (buf + IDX_DATA);
     float    *p_float32   = (float*)(buf + IDX_DATA);
     double   val;
     uint32_t service_mask = buf[IDX_SVC_MASK];
     uint32_t valid_mask   = buf[IDX_VALID_MASK(size)];
     uint64_t pulse_id     = ((uint64_t)(buf[IDX_PIDU])) << 32 | buf[IDX_PIDL];

     epicsTimeStamp _ts;
     _ts.nsec = buf[IDX_NSEC];
     _ts.secPastEpoch  = buf[IDX_SEC];
     setTimeStamp(&_ts);    // timestamp update for asyn port and related PVs


     bsssList_t *plist = (bsssList_t *) ellFirst(pBsssEllList);
     int chn_mask = 0x1;
     int data_chn = 0;
     while(plist) {
         for(int i = 0, svc_mask = 0x1; i < NUM_BSSS_CHN; i++, svc_mask <<= 1) {
             if(service_mask & svc_mask) {
                 setInteger64Param(plist->p_bsssPID[i], pulse_id);  // pulse id update if service mask is set

                 setDoubleParam(plist->p_bsss[i], INFINITY);   // make asyn PV update even posting the same value with previous

                 if(valid_mask & chn_mask) {  // data update for valid mask
                     switch(plist->type){
                         case int32_bsss:
                             val = (double) (p_int32[data_chn]);
                             break;
                         case uint32_bsss:
                             val = (double) (p_uint32[data_chn]);
                             break;
                         case float32_bsss:
                             val = (double) (p_float32[data_chn]);
                             break;
                         case uint64_bsss:
                         default:
                              val = NAN;   // uint64 never defined
                             break;
                     }
                 } else val = NAN;  // put NAN for invalid mask

                 if(!isnan(val)) val = val * (*plist->pslope) + (*plist->poffset);
                 setDoubleParam(plist->p_bsss[i], val);
             }
         }

         plist = (bsssList_t *) ellNext(&plist->node);  // evolve to next data channel
         chn_mask <<= 1;  // evolve mask bit to next data channel
         data_chn++;      // evolve data channel number to next channel
     }
     

     callParamCallbacks();

}

extern "C" {

static int  bsssMonitorPoll(void)
{
    while(1) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pBsssDrv) p->pBsssDrv->MonitorStatus();
            p = (pDrvList_t *) ellNext(&p->node);
        }
        epicsThreadSleep(1.);
    }

    return 0;
}


int bsssAssociateBsaChannels(const char *port_name)
{
    return associateBsaChannels(port_name);
}



int bsssAsynDriverConfigure(const char *portName, const char *reg_path, const char *named_root)
{
    pDrvList_t *pl = find_drvByPort(portName);
    if(!pl) {
        pl = find_drvLast();
        if(pl) {
            if(!pl->port_name && !pl->named_root && !pl->pBsssDrv) pl->port_name = epicsStrDup(portName);
            else pl = NULL;
        }
    }
    if(!pl) {
        printf("BSSS list never been configured for port (%s)\n", portName);
        return -1;
    }

    pl->named_root = (named_root && strlen(named_root))?epicsStrDup(named_root): cpswGetRootName();

    if(!pl->pBsssEllList) return -1;

    int i = 0;
    bsssList_t *p = (bsssList_t *) ellFirst(pl->pBsssEllList);
    while(p) {
        i += (int) (&p->p_lastParam - &p->p_firstParam -1);
        p = (bsssList_t *) ellNext(&p->node);

    }    /* calculate number of dyanmic parameters */

    pl->port_name = epicsStrDup(portName);
    pl->reg_path  = epicsStrDup(reg_path);
    pl->pBsssDrv  = new bsssAsynDriver(portName, reg_path, i, pl->pBsssEllList, pl->named_root);

    return 0;
}


static const iocshArg initArg0 = { "portName",                                           iocshArgString };
static const iocshArg initArg1 = { "register path (which should be described in yaml):", iocshArgString };
static const iocshArg initArg2 = { "named_root (optional)",                              iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = { "bsssAsynDriverConfigure", 3, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
 
    bsssAsynDriverConfigure(args[0].sval,  /* port name */
                           args[1].sval,  /* register path */
                           (args[2].sval && strlen(args[2].sval))? args[2].sval: NULL);  /* named_root */
}


static const iocshArg associateArg0 = { "bsa port", iocshArgString };
static const iocshArg * const associateArgs [] = { &associateArg0 };
static const iocshFuncDef associateFuncDef = { "bsssAssociateBsaChannels", 1, associateArgs };
static void associateCallFunc(const iocshArgBuf *args)
{

    associateBsaChannels(args[0].sval);
}

void bsssAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,        initCallFunc);
    iocshRegister(&associateFuncDef,   associateCallFunc);
}

epicsExportRegistrar(bsssAsynDriverRegister);




/* EPICS driver support for bsssAsynDriver */

static int bsssAsynDriverReport(int interest);
static int bsssAsynDriverInitialize(void);

static struct drvet bsssAsynDriver = {
    2,
    (DRVSUPFUN) bsssAsynDriverReport,
    (DRVSUPFUN) bsssAsynDriverInitialize
};

epicsExportAddress(drvet, bsssAsynDriver);

static int bsssAsynDriverReport(int interest)
{
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        printf("named_root: %s, port: %s, driver instace: %p, number of BSSS varibles: %d\n",
              (p->named_root && strlen(p->named_root))?p->named_root: "Unknown",
              (p->port_name && strlen(p->port_name))? p->port_name: "Unknown",
              p->pBsssDrv,
              (p->pBsssEllList)? ellCount(p->pBsssEllList): -1);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int bsssAsynDriverInitialize(void)
{

   /* Implement EPICS driver initialization here */
    init_drvList();

    if(!pDrvEllList) {
        printf("BSSS Driver never been configured\n");
        return 0;
    }

    printf("BSSS driver: %d of bsss driver instance(s) has (have) been configured\n", ellCount(pDrvEllList));

    epicsThreadCreate("bsssDrvPoll", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) bsssMonitorPoll, 0);

    return 0;
}


} /* extern "C" */

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
    sprintf(param_name, EDEFENABLE_STR);       createParam(param_name, asynParamInt32, &p_edefEnable);

    for(int i = 0; i < NUM_BSSS_CHN; i++) {
        sprintf(param_name, RATEMODE_STR, i);  createParam(param_name, asynParamInt32, &p_rateMode[i]);
        sprintf(param_name, FIXEDRATE_STR, i); createParam(param_name, asynParamInt32, &p_fixedRate[i]);
        sprintf(param_name, ACRATE_STR, i);    createParam(param_name, asynParamInt32, &p_acRate[i]);
        sprintf(param_name, TSLOTMASK_STR, i); createParam(param_name, asynParamInt32, &p_tSlotMask[i]);
        sprintf(param_name, EXPSEQNUM_STR, i); createParam(param_name, asynParamInt32, &p_expSeqNum[i]);
        sprintf(param_name, EXPSEQBIT_STR, i); createParam(param_name, asynParamInt32, &p_expSeqBit[i]);
        sprintf(param_name, DESTMODE_STR, i);  createParam(param_name, asynParamInt32, &p_destMode[i]);
        sprintf(param_name, DESTMASK_STR, i);  createParam(param_name, asynParamInt32, &p_destMask[i]);
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
        case 0:  /* Inclusion */
            pBsss->setDestInclusion(chn, destMask);
            break;
        case 1:  /* Exclusion */
            pBsss->setDestExclusion(chn, destMask);
            break;
        case 2:  /* Disable */
            pBsss->setDestDisable(chn);
            break;
        default:  /* nothing to do */
            break;
    }
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
    else if(function == p_edefEnable)
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
    }

    done:
    callParamCallbacks();
    return status;
}



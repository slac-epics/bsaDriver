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
#include <bsasAsynDriver.h>
#include <bsaAsynDriver.h>

#include <yamlLoader.h>


static const char *driverName = "bsasAsynDriver";

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE           node;
    char              *named_root;
    char              *port_name;
    char              *reg_path;
    bsasAsynDriver    *pBsasDrv;
    ELLLIST           *pBsasEllList;
} pDrvList_t;

static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsasAsynDriver(init_drvList)");
        ellInit(pDrvEllList);
    }

    return;
}

static pDrvList_t *find_drvLast(void)
{
    init_drvList();

    if(ellCount(pDrvEllList)) return (pDrvList_t *) ellLast(pDrvEllList);
    else                       return (pDrvList_t *) NULL;
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

    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsasAsynDriver(prep_drvAnonimous)");

    p->named_root   = NULL;
    p->port_name    = NULL;
    p->reg_path     = NULL;
    p->pBsasDrv     = NULL;
    p->pBsasEllList = NULL;

    ellAdd(pDrvEllList, &p->node);
    return ellCount(pDrvEllList);
}


static int bsasAdd(const char *bsasKey, bsasDataType_t type, double *slope, double *offset)
{
    pDrvList_t * pl = find_drvLast();

    if(pl) {
        if(pl->pBsasDrv || (pl->port_name && strlen(pl->port_name))) pl = NULL;      /* the driver node has been configure.
                                                                                        need to make another one */
    }

    while(!pl) {
        prep_drvAnonimous();
        pl = find_drvLast();
    }

    while(!pl->pBsasEllList) {
        pl->pBsasEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsasAsynDriver (bsasAdd)");
        ellInit(pl->pBsasEllList);
    }

    bsasList_t *p = (bsasList_t *) mallocMustSucceed(sizeof(bsasList_t) , "bsasAsynDriver (bsasAdd)");
    strcpy(p->bsas_name, bsasKey);
    for(int i = 0; i < NUM_BSAS_MODULES; i++) {
        p->p_ts[i] = -1;    p->pname_ts[i][0]  = '\0';
        p->p_pid[i] = -1;   p->pname_pid[i][0] = '\0';
        p->p_cnt[i] = -1;   p->pname_cnt[i][0] = '\0';
        p->p_avg[i] = -1;   p->pname_avg[i][0] = '\0';
        p->p_rms[i] = -1;   p->pname_rms[i][0] = '\0';
        p->p_min[i] = -1;   p->pname_min[i][0] = '\0';
        p->p_max[i] = -1;   p->pname_max[i][0] = '\0';
    }

    p->type    = type;
    p->pslope  = slope;
    p->poffset = offset;

    ellAdd(pl->pBsasEllList, &p->node);

    return 0;
}



static int associateBsasChannels(const char *port_name)
{
    ELLLIST *pBsaEllList = find_bsaChannelList(port_name);

    if(!pBsaEllList) return -1;
    bsaList_t *p = (bsaList_t *) ellFirst(pBsaEllList);

    while(p) {
        bsasAdd(p->bsa_name, bsasDataType_t(p->type), &p->slope, &p->offset);
        p = (bsaList_t *) ellNext(&p->node);
    }

    printf("Associate %d of channels from bsa port (%s) \n", ellCount(find_drvLast()->pBsasEllList), port_name);

    return 0;

}


static void bsas_callback(void *pUsr, void *buf, unsigned size)
{
    ((bsasAsynDriver *) pUsr)->bsasCallback(buf, size);
}



bsasAsynDriver::bsasAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsasEllList,  const char *named_root)
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
    if(!pBsasEllList || !ellCount(pBsasEllList)) return;   /* if there is no Bsss data channels in the list, nothing to do */
    this->pBsasEllList = pBsasEllList;

    Path root_ = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
    if(!root_) {
        printf("BSAS driver: could not find root path\n");
        return;
    }

    Path reg_bsas = root_->findByName(reg_path);
    if(!reg_path) {
        printf("BSAS driver: could not find register at pth %s\n", reg_path);
        return;
    }

    for(int i = 0; i < NUM_BSAS_MODULES; i++) {
        char module_name[64];
        sprintf(module_name, "BsasModule[%d]", i);
        Path reg_module = reg_bsas->findByName(module_name);

        if(!reg_module) {
            printf("BSAS driver: could not find module %s\n", module_name);
            return;
        }
        this->pBsas[i] = new Bsas::BsasModuleYaml(reg_bsas);  /* create API interface */
    }

    SetupAsynParams();
    registerBsasCallback(named_root, bsas_callback, (void *) this);
}

asynStatus bsasAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";

    /* set the parameter in the parameter library */
    status = (asynStatus) setIntegerParam(function, value);

    for(int i = 0; i < NUM_BSAS_MODULES; i++) {  /* seeking modules */
        for(int j = 0; j < NUM_BSAS_CTRL; j++) {    /* seeking CTRLs */
            if(function == p_module[i].idx.p_idx[j].p_edefEnable) {
                EdefEnable(i, ctrlIdx_t(j), value);
                goto done;
            }
            else if(function == p_module[i].idx.p_idx[j].p_rateMode ||
                    function == p_module[i].idx.p_idx[j].p_fixedRate  ||
                    function == p_module[i].idx.p_idx[j].p_acRate     ||
                    function == p_module[i].idx.p_idx[j].p_tSlotMask  ||
                    function == p_module[i].idx.p_idx[j].p_expSeqNum  ||
                    function == p_module[i].idx.p_idx[j].p_expSeqBit) {
                 SetRate(i, ctrlIdx_t(j));
                 goto done;
             }
            
        }  /* seeking CTRLs */
    }  /* seeking modules */

    done:
    callParamCallbacks();
    return status;
}


void bsasAsynDriver::SetupAsynParams(void)
{
    char param_name[64];
    const char *param_str[] = { (const char *)PARAM_ACQUIRE_STR,
                                (const char *)PARAM_ROWADV_STR,
                                (const char *)PARAM_TBLRESET_STR };

    for(int i = 0; i < NUM_BSAS_MODULES; i++) {
        for(int j = 0; j < sizeof(param_str); j++) {
            sprintf(param_name, EDEFENABLE_STR, i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_edefEnable);
            sprintf(param_name, RATEMODE_STR,   i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_rateMode);
            sprintf(param_name, FIXEDRATE_STR,  i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_fixedRate);
            sprintf(param_name, ACRATE_STR,     i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_acRate);
            sprintf(param_name, TSLOTMASK_STR,  i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_tSlotMask);
            sprintf(param_name, EXPSEQNUM_STR,  i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_expSeqNum);
            sprintf(param_name, EXPSEQBIT_STR,  i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_expSeqBit);
            sprintf(param_name, DESTMODE_STR,   i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_destMode);
            sprintf(param_name, DESTMASK_STR,   i, param_str[j]); createParam(param_name, asynParamInt32, &p_module[i].idx.p_idx[j].p_destMask);
        }
    }
}

void bsasAsynDriver::SetRate(int module, ctrlIdx_t ctrl)
{
    uint32_t rateMode, fixedRate, acRate, tSlotMask, expSeqNum, expSeqBit;

    getIntegerParam(p_module[module].idx.p_idx[ctrl].p_rateMode, (epicsInt32*) &rateMode);
    switch(rateMode) {
        case 0:  /* fixed rate mode */
            getIntegerParam(p_module[module].idx.p_idx[ctrl].p_fixedRate, (epicsInt32 *) &fixedRate);
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->SetFixedRate(fixedRate);
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->SetFixedRate(fixedRate);
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->SetFixedRate(fixedRate);
                    break;
                default: break;
            }
            break;
        case 1:  /* AC rate mode */
            getIntegerParam(p_module[module].idx.p_idx[ctrl].p_acRate, (epicsInt32 *) &acRate);
            getIntegerParam(p_module[module].idx.p_idx[ctrl].p_tSlotMask, (epicsInt32 *)  &tSlotMask);
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->SetACRate(tSlotMask, acRate);
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->SetACRate(tSlotMask, acRate);
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->SetACRate(tSlotMask, acRate);
                    break;
                default: break;
            }
            break;
        case 2:  /* Seq Rate mode */
            getIntegerParam(p_module[module].idx.p_idx[ctrl].p_expSeqNum, (epicsInt32 *) &expSeqNum);
            getIntegerParam(p_module[module].idx.p_idx[ctrl].p_expSeqBit, (epicsInt32 *) &expSeqBit);
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->SetSeqBit(expSeqNum, expSeqBit);
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->SetSeqBit(expSeqNum, expSeqBit);
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->SetSeqBit(expSeqNum, expSeqBit);
                    break;
                default: break;
            }
            break;
         default:  /* nothing to do */
            break;
    }
}

void bsasAsynDriver::SetDest(int module, ctrlIdx_t ctrl)
{
    uint32_t destMode, destMask;

    getIntegerParam(p_module[module].idx.p_idx[ctrl].p_destMode,  (epicsInt32 *) &destMode);
    getIntegerParam(p_module[module].idx.p_idx[ctrl].p_destMask, (epicsInt32 *) &destMask);

    switch(destMode) {
        case 2:     /* Includesion */
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->SetInclusionMask(destMask);
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->SetInclusionMask(destMask);
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->SetInclusionMask(destMask);
                    break;
                default: break;
            }
            break;
        case 1:     /* Exclusion */
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->SetExclusionMask(destMask);
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->SetExclusionMask(destMask);
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->SetExclusionMask(destMask);
                    break;
                default: break;
            }
            break;
        case 0:     /* Disable */
            switch(ctrl) {
                case acquire_idx:
                    pBsas[module]->pAcquire->DisableDestination();
                    break;
                case rowAdvance_idx:
                    pBsas[module]->pRowAdvance->DisableDestination();
                    break;
                case tableReset_idx:
                    pBsas[module]->pTableReset->DisableDestination();
                    break;
                default: break;
            }
            break;
        default:    /* nothing to do */
            break;
    }

}


void bsasAsynDriver::EdefEnable(int module, ctrlIdx_t ctrl, int enable)
{
    switch(ctrl) {
        case acquire_idx:
            pBsas[module]->pAcquire->Enable(enable);
            break;
        case rowAdvance_idx:
            pBsas[module]->pRowAdvance->Enable(enable);
            break;
        case tableReset_idx:
            pBsas[module]->pTableReset->Enable(enable);
            break;
        default:
            break;
    }
}


void bsasAsynDriver::bsasCallback(void *p, unsigned size)
{
}





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

#include <pvxs/server.h>
#include <pvxs/sharedpv.h>
#include <pvxs/log.h>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>


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
    p->pv_name[0] = '\0';
    p->swChIndex = -1;
    p->hwChIndex = -1;
    p->p_channelMask = -1;
    p->p_channelSevr = -1;
    for(int i = 0; i < NUM_BSAS_MODULES; i++) {
        p->p_ts[i] = -1;
        p->p_pid[i] = -1;
        p->p_cnt[i] = -1;
        p->p_avg[i] = -1;
        p->p_rms[i] = -1;
        p->p_min[i] = -1;
        p->p_max[i] = -1;
        p->p_val[i] = -1;
    }

    p->pname_ts[0]  = '\0';
    p->pname_pid[0] = '\0';
    p->pname_cnt[0] = '\0'; p->fname_cnt[0] = '\0';
    p->pname_avg[0] = '\0'; p->fname_avg[0] = '\0';
    p->pname_rms[0] = '\0'; p->fname_rms[0] = '\0';
    p->pname_min[0] = '\0'; p->fname_min[0] = '\0';
    p->pname_max[0] = '\0'; p->fname_max[0] = '\0';
    p->pname_val[0] = '\0'; p->fname_val[0] = '\0';

    p->type    = type;
    p->pslope  = slope;
    p->poffset = offset;

    ellAdd(pl->pBsasEllList, &p->node);

    return 0;
}

static int bsasPVName(const char *bsasKey, const char *pv_name)
{
    pDrvList_t *pl = find_drvLast();

    if(pl) {
        if(pl->pBsasDrv || (pl->port_name && strlen(pl->port_name))) pl = NULL;
    }

    if(!pl) {
        printf("No BSAS data channel list is available\n");
        return 0;
    }

    if(!pl->pBsasEllList) {
        printf("Internal issue on the BSAS data channe list\n");
        return 0;
    }

    bsasList_t *p = (bsasList_t *) ellFirst(pl->pBsasEllList);
    while(p) {
        if(!strcmp(p->bsas_name, bsasKey)) break;
        p = (bsasList_t *) ellNext(&p->node);
    }

    if(!p) {
        printf("Could not find the BSAS name: (%s)\n", bsasKey);
        return 0;
    }

    if(p->pv_name[0]) {
        printf("The BSAS (%s) already has PV name(%s)\n", p->bsas_name, p->pv_name);
        return 0;
    }

    sprintf(p->pv_name,   "%s",     pv_name);
    sprintf(p->pname_ts,  "%s.TS",  pv_name);
    sprintf(p->pname_pid, "%s.PID", pv_name);
    sprintf(p->pname_cnt, "%s.CNT", pv_name);
    sprintf(p->pname_avg, "%s.AVG", pv_name);
    sprintf(p->pname_rms, "%s.RMS", pv_name);
    sprintf(p->pname_min, "%s.MIN", pv_name);
    sprintf(p->pname_max, "%s.MAX", pv_name);
    sprintf(p->pname_val, "%s.VAL", pv_name);

    return 0;
}



static int associateBsaChannels(const char *port_name)
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



void chnCol::init(void)
{
    for(int i = 0; i < MAXROWS; i++) {
        cnt[i] = 0;
        val[i] = NAN;
        avg[i] = NAN;
        rms[i] = NAN;
        min[i] = NAN;
        max[i] = NAN;
    }
}

int chnCol::store(int row, uint32_t cnt, double val, double avg, double rms, double min, double max)
{
    if(row< 0 || row > MAXROWS-1) return -1;

    this->last_row = row;

    this->cnt[row] = cnt;
    this->val[row] = val;
    this->avg[row] = avg;
    this->rms[row] = rms;
    this->min[row] = min;
    this->max[row] = max;

    return 0;
}

void chnCol::pushPV(pvxs::Value *ppv, void *pChn)
{
    bsasList_t *p = (bsasList_t *) pChn;

    (*ppv)[VALUE_FIELD][p->fname_cnt] = pvxs::shared_array<const uint32_t>(cnt, cnt + last_row + 1);
    (*ppv)[VALUE_FIELD][p->fname_val] = pvxs::shared_array<const double>  (val, val + last_row + 1);
    (*ppv)[VALUE_FIELD][p->fname_avg] = pvxs::shared_array<const double>  (avg, avg + last_row + 1);
    (*ppv)[VALUE_FIELD][p->fname_rms] = pvxs::shared_array<const double>  (rms, rms + last_row + 1);
    (*ppv)[VALUE_FIELD][p->fname_min] = pvxs::shared_array<const double>  (min, min + last_row + 1);
    (*ppv)[VALUE_FIELD][p->fname_max] = pvxs::shared_array<const double>  (max, max + last_row + 1);
}

ntTbl::ntTbl(int num_chn)
{
    this->num_chn = num_chn;
    col = new chnCol[num_chn];    // allocate columns
    init();                       // initialize all of the columns
}

void ntTbl::init(void)
{
    for(int i = 0; i < num_chn; i++) (col+i)->init();
    for(int row = 0; row < MAXROWS; row++) {
//        this->timestmap[row] = 0;
        this->sec[row] = 0;
        this->nsec[row] = 0;
        this->pulse_id[row] = 0;
    }
}

int ntTbl::store(int row, uint64_t timestamp, uint64_t pulse_id)
{
    typedef struct {
        uint32_t nsec;
        uint32_t  sec;
    } ts_t;

    ts_t *p = (ts_t *) &timestamp;

    if(row <0 || row > MAXROWS-1) return -1;

    this->last_row       = row;
//    this->timestamp[row] = timestamp;
    this->sec[row]       = p->sec;
    this->nsec[row]      = p->nsec;
    this->pulse_id[row]  = pulse_id;

    return 0;
}

int ntTbl::store(int col, int row, uint32_t cnt, double val, double avg, double rms, double min, double max)
{
    if(col<0 || col > num_chn-1) return -1;

    return (this->col + col)->store(row, cnt, val, avg, rms, min, max);


}


void ntTbl::pushPV(pvxs::Value *ppv, std::vector<void *> *pActiveChannels)
{
    int col = 0;
    (*ppv)[VALUE_FIELD][SEC_COL]  = pvxs::shared_array<const uint32_t>(sec,      sec+last_row+1);
    (*ppv)[VALUE_FIELD][NSEC_COL] = pvxs::shared_array<const uint32_t>(nsec,     nsec+last_row+1);
    (*ppv)[VALUE_FIELD][PID_COL]  = pvxs::shared_array<const uint64_t>(pulse_id, pulse_id+last_row+1);

    for(auto it = pActiveChannels->begin(); it < pActiveChannels->end(); it++) {
        (this->col + (col++))->pushPV(ppv, *it);
    }
}



edefNTTbl::edefNTTbl(int num_chn)
{
    table_count = -1;
                // indicator for active buffer
    swing_idx   = 0;
                // implement swing buffer one for active, the other one for standby
    pTbl[0] = new ntTbl(num_chn);
    pTbl[1] = new ntTbl(num_chn);
}


pvxs::server::SharedPV edefNTTbl::lateInit(const char *ntTableName, std::vector <void *> *pActiveChannels)
{
    auto labels = std::vector<std::string>();
    auto value  = pvxs::TypeDef(pvxs::TypeCode::Struct, {});

    pv = pvxs::server::SharedPV::buildReadonly();

    value += {pvxs::Member(pvxs::TypeCode::UInt32A, SEC_COL)};    labels.push_back(SEC_COL);
    value += {pvxs::Member(pvxs::TypeCode::UInt32A, NSEC_COL)};   labels.push_back(NSEC_COL);
    value += {pvxs::Member(pvxs::TypeCode::UInt64A, PID_COL)};    labels.push_back(PID_COL);


    for(auto it = pActiveChannels->begin(); it < pActiveChannels->end(); it++) {
        bsasList_t *p = (bsasList_t *) (*it);
        
        value += {pvxs::Member(pvxs::TypeCode::UInt32A,  p->fname_cnt)}; labels.push_back(p->pname_cnt);
        value += {pvxs::Member(pvxs::TypeCode::Float64A, p->fname_val)}; labels.push_back(p->pname_val);
        value += {pvxs::Member(pvxs::TypeCode::Float64A, p->fname_avg)}; labels.push_back(p->pname_avg);
        value += {pvxs::Member(pvxs::TypeCode::Float64A, p->fname_rms)}; labels.push_back(p->pname_rms);
        value += {pvxs::Member(pvxs::TypeCode::Float64A, p->fname_min)}; labels.push_back(p->pname_min);
        value += {pvxs::Member(pvxs::TypeCode::Float64A, p->fname_max)}; labels.push_back(p->pname_max);
        
    }

    _labels = pvxs::shared_array<const std::string>(labels.begin(), labels.end());
    _def = pvxs::TypeDef(pvxs::TypeCode::Struct, NTTBL_ID, {
                         pvxs::members::StringA(LABEL_FIELD),
                         value.as(VALUE_FIELD)
                         });
    
    _initial              = _def.create();
    _initial[LABEL_FIELD] = _labels;

    pv.open(_initial);

    return pv;
}

void edefNTTbl::pushPV(std::vector<void *> *pActiveChannels)
{
    int idx = swing_idx?0:1;
    
    pTbl[idx]->pushPV(&_initial, pActiveChannels);
    pv.post(_initial);
}

inline void edefNTTbl::swing(void)
{
    // switch active and standby
    swing_idx = swing_idx?0:1;
    pTbl[swing_idx]->init();
}


int edefNTTbl::checkUpdate(int table_count)
{

    if(this->table_count == -1) this->table_count = table_count;
    else
    if(this->table_count != table_count) {
          this->table_count = table_count;
          return -1; 
    }

    return 0;
}

int edefNTTbl::store(int row, uint64_t timestamp, uint64_t pulse_id)
{
    return pTbl[swing_idx]->store(row, timestamp, pulse_id);
}

int edefNTTbl::store(int col, int row, uint32_t cnt, double val, double avg, double rms, double min, double max)
{
                // store data into active buffer
    return pTbl[swing_idx]->store(col, row, cnt, val, avg, rms, min, max);
    
}

bsasAsynDriver::bsasAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsasEllList,
                               const char *ntTable_name1,
                               const char *ntTable_name2,
                               const char *ntTable_name3,
                               const char *ntTable_name4,
                               const char *named_root)
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
        this->pBsas[i] = new Bsas::BsasModuleYaml(reg_module);  /* create API interface */
    }

    // Initialize hardware/software channel map
    for (int i = 0; i < HW_CHANNELS; i++)
        bsasHwChannelUsage[i] = {};

    strcpy(ntTableName[0], ntTable_name1);
    strcpy(ntTableName[1], ntTable_name2);
    strcpy(ntTableName[2], ntTable_name3);
    strcpy(ntTableName[3], ntTable_name4);

    this->channelMask = 0;
    this->channelSevr = 0;
    activeChannels = new std::vector<void *>;
    bsasList_t *p = (bsasList_t *) ellFirst(this->pBsasEllList);
    while(p) {
        if(p->pv_name[0]) {
            this->channelMask |= (0x1)  << p->hwChIndex;
            sprintf(p->fname_cnt, CNT_STR, p->swChIndex);
            sprintf(p->fname_val, VAL_STR, p->swChIndex);
            sprintf(p->fname_avg, AVG_STR, p->swChIndex);
            sprintf(p->fname_rms, RMS_STR, p->swChIndex);
            sprintf(p->fname_min, MIN_STR, p->swChIndex);
            sprintf(p->fname_max, MAX_STR, p->swChIndex);
            activeChannels->push_back((void *) p);
            bsasHwChannelUsage[p->hwChIndex].push_back(p->swChIndex);
        }
        p = (bsasList_t *) ellNext(&p->node);
    }
    //printMap();
    
    for(int i =0; i < NUM_BSAS_MODULES; i++) pEdefNTTbl[i] = new edefNTTbl(activeChannels->size());

    SetupAsynParams();
    registerBsasCallback(named_root, bsas_callback, (void *) this);
    SetChannelMask(this->channelMask);
}


bsasAsynDriver::~bsasAsynDriver()
{
}

void bsasAsynDriver::printMap()
{
    for(int i=0;i<HW_CHANNELS;i++)
    {
        printf("| %d | -> ",i);
        for(auto x:bsasHwChannelUsage[i])
        {
            printf("%d, ",x);
        }
        printf("\n");
    }
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
             else if(function == p_module[i].idx.p_idx[j].p_destMode ||
                     function == p_module[i].idx.p_idx[j].p_destMask) {
                 SetDest(i, ctrlIdx_t(j));
                 goto done;
             }
            
        }  /* seeking CTRLs */
    }  /* seeking modules */

    for(std::vector<void*>::iterator  it = activeChannels->begin(); it != activeChannels->end(); it++) {
        bsasList_t *p = (bsasList_t *) (*it);
        if(function == p->p_channelMask) 
        {
            if (value) // enable hardware channel
            {
                // Search for software channel in the channel map
                auto vec = bsasHwChannelUsage[p->hwChIndex];
                if (std::find(vec.begin(),vec.end(),p->swChIndex) == vec.end())
                {
                    // Only set the hardware channel bit if not set before
                    if (bsasHwChannelUsage[p->hwChIndex].size() == 0)
                        SetChannelMask(p->hwChIndex, ENABLE);
                    
                    // Add software channel to the usage list
                    bsasHwChannelUsage[p->hwChIndex].push_back(p->swChIndex);
                }
            }
            else // disable hardware channel
            {
                // Search for software channel in the channel map
                auto pos = std::find(bsasHwChannelUsage[p->hwChIndex].begin(),bsasHwChannelUsage[p->hwChIndex].end(),p->swChIndex);
                if (pos != bsasHwChannelUsage[p->hwChIndex].end())
                {
                    // Remove software channel from the usage list
                    bsasHwChannelUsage[p->hwChIndex].erase(pos);
                    
                    // Disable hardware channel only if not used by any other software channels
                    if (bsasHwChannelUsage[p->hwChIndex].size() == 0)
                        SetChannelMask(p->hwChIndex, DISABLE);
                }
            }
            goto done;
        }
        if(function == p->p_channelSevr) {
            SetChannelSevr(p->hwChIndex, value);
            goto done;
        }
    }


    done:
    callParamCallbacks();
    return status;
}


void bsasAsynDriver::SetupAsynParams(void)
{
    char param_name[140];
    const char *param_str[] = { (const char *)PARAM_ACQUIRE_STR,
                                (const char *)PARAM_ROWADV_STR,
                                (const char *)PARAM_TBLRESET_STR };

    for(int i = 0; i < NUM_BSAS_MODULES; i++) {
        for(int j = 0; j < sizeof(param_str)/sizeof(const char *); j++) {
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


    for(std::vector<void *>::iterator it = activeChannels->begin(); it != activeChannels->end(); it++) {
        bsasList_t *p = (bsasList_t *) (*it);
        sprintf(param_name, CHANNEL_MASK_STR, p->bsas_name); createParam(param_name, asynParamInt32, &(p->p_channelMask));
        sprintf(param_name, CHANNEL_SEVR_STR, p->bsas_name); createParam(param_name, asynParamInt32, &(p->p_channelSevr));
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


void bsasAsynDriver::SetChannelMask(int chn, bool flag)
{
    channelMask &= ~(uint32_t(0x1) << chn);    // clear bit location
    channelMask |=  (uint32_t(flag?0x1:0x0) << chn);  // set bit location

    SetChannelMask(channelMask);
}

void bsasAsynDriver::SetChannelMask(uint32_t mask)
{
    for(int module = 0; module < NUM_BSAS_MODULES; module++) {
        pBsas[module]->SetChannelMask(mask);
        pBsas[module]->Enable(mask?1:0);
    }

}

void bsasAsynDriver::SetChannelSevr(int chn, uint64_t sevr)
{
    channelSevr &= ~(uint64_t(0x3) << (chn*2));
    channelSevr |= ((uint64_t(0x3) & sevr) << (chn*2));

    SetChannelSevr(channelSevr);
}

void bsasAsynDriver::SetChannelSevr(uint64_t sevr)
{
    for(int module = 0; module < NUM_BSAS_MODULES; module++) {
        pBsas[module]->SetChannelSeverity(sevr);
    }
}



void bsasAsynDriver::lateInit(void)
{
    static auto server = pvxs::ioc::server();

    for(int i = 0; i < NUM_BSAS_MODULES; i++)  {
        server.addPV(ntTableName[i], pEdefNTTbl[i]->lateInit(ntTableName[i], activeChannels));

   }
}

void bsasAsynDriver::bsasCallback(void *p, unsigned size)
{
    packet_t     *pk = (packet_t *) p ;
    header_t     *hd = &pk->hd;
    payload_t    *pl = pk->pl;

    int i   = 0;
    int col = 0;
    
    uint32_t cnt, tmpVal, mask;
    double   val, avg, rms, min, max, sq;
    union {
        uint32_t u32;
        int32_t  i32;
        float    f32;
        uint64_t u64;
        int64_t  i64;
    }  _val, _sum, _min, _max, _sum_square;

    if(pEdefNTTbl[hd->edef_index]->checkUpdate(hd->table_count)) {
        pEdefNTTbl[hd->edef_index]->swing();  // swing buffer
        pEdefNTTbl[hd->edef_index]->pushPV(activeChannels);
    }

    pEdefNTTbl[hd->edef_index]->store(hd->row_number, hd->timestamp, hd->pulse_id);

     // Variable to keep track of bit boundaries as we read in the packet payload
     unsigned bitSum = 0;

     // Fault flag to indicate violation of bit boundaries in the 
     // association of the active channels/PVs vs the packet payload
     bool userFault = false;

     // Incoming payload data are 32-bit words
     const unsigned wordWidth = BLOCK_WIDTH_32;

     for(std::vector<void*>::iterator it = activeChannels->begin(); it != activeChannels->end(); it++, col++) {
        bsasList_t *plist = (bsasList_t *)(*it);
        _val.u32        = (pl+i)->val;
        _sum.u32        = (pl+i)->sum;
        _sum_square.u64 = (pl+i)->sum_square;
        _min.u32        = (pl+i)->min;
        _max.u32        = (pl+i)->max;

        // If the corresponding hardare channel is disabled, continue to next iteration
        if (!(this->channelMask & (uint32_t(0x1) << plist->hwChIndex)))
            continue;

        switch(plist->type) {
            case uint2_bsas:
                // Read unpartitioned payload data for current channel/PV  
                tmpVal = _val.u32;
                // Bit-shift and extract channel/PV value with a mask
                mask = KEEP_LSB_2; tmpVal >>= bitSum; tmpVal &= mask;
                // Assign extracted channel/PV value and unchanged statistics
                val = double(tmpVal);
                avg = double(_sum.u32);
                rms = double(_sum_square.u64);
                min = double(_min.u32);
                max = double(_max.u32);
                // Increment bitSum counter
                bitSum += BLOCK_WIDTH_2;
                break;
            case int16_bsas:
            case uint16_bsas:
                // Read unpartitioned payload data for current channel/PV  
                tmpVal = _val.u32;
                // Bit-shift and extract channel/PV value with a mask
                mask = KEEP_LSB_16; tmpVal >>= bitSum; tmpVal &= mask;
                // Assign extracted channel/PV value and unchanged statistics
                val = double(tmpVal);
                avg = double(_sum.u32);
                rms = double(_sum_square.u64);
                min = double(_min.u32);
                max = double(_max.u32);
                // Increment bitSum counter
                bitSum += BLOCK_WIDTH_16;
                break;
            case int32_bsas:
                // Assign extracted channel/PV value and statistics
                val = double(_val.i32);
                avg = double(_sum.i32);
                rms = double(_sum_square.i64);
                min = double(_min.i32);
                max = double(_max.i32);
                // Increment bitSum counter
                bitSum += BLOCK_WIDTH_32;
                break;
            case uint32_bsas:
                // Assign extracted channel/PV value and statistics
                val = double(_val.u32);
                avg = double(_sum.u32);
                rms = double(_sum_square.u64);
                min = double(_min.u32);
                max = double(_max.u32);
                // Increment bitSum counter
                bitSum += BLOCK_WIDTH_32;
                break;
            case float32_bsas:
                // Assign extracted channel/PV value and statistics
                val = double(_val.f32);
                avg = double(_sum.f32);
                rms = double(_sum_square.f32);
                min = double(_min.f32);
                max = double(_max.f32);
                // Increment bitSum counter
                bitSum += BLOCK_WIDTH_32;
                break;
            case uint64_bsas:
            default:
                val = avg = rms = min = max = NAN;
                break;
        }

        cnt = (pl+i)->sample_count;

        if((pl+i)->flag_fixed || cnt < 2 ) {   // no-statistics
            cnt = 1;
            rms = 0.;
            min = max = avg = val;
            goto done;
        }
        if((pl+i)->exception_sum) {   // exception on sum
            avg = NAN;
        }
        if((pl+i)->exception_var) {   // exception on sum of sequares
            rms = NAN;
        }

        if(cnt >1) {
            if(avg != NAN) {
                sq = avg * avg;
                avg /= cnt;
            } else sq = NAN;

            if(sq != NAN && rms != NAN) rms = sqrt((rms - sq / cnt) / (cnt -1));
            else                        rms = NAN;
        }

        done:    // linear conversion
        if(val != NAN) val = val * (*plist->pslope) + (*plist->poffset);
        if(avg != NAN) avg = avg * (*plist->pslope) + (*plist->poffset);
        if(rms != NAN) rms = rms * (*plist->pslope);
        if(min != NAN) min = min * (*plist->pslope) + (*plist->poffset);
        if(max != NAN) max = max * (*plist->pslope) + (*plist->poffset);

       pEdefNTTbl[hd->edef_index]->store(col, hd->row_number, cnt, val, avg, rms, min, max);

       // Increment index only if done reading current 32-bit word from the packet payload 
       if (bitSum == wordWidth)
       {
           // All good, move on to the next 32-bit word
           bitSum = 0; // reset bitSum counter
           i++;        // point to next 32-bit word in packet payload 
       }
       else if (bitSum > wordWidth)
           userFault = true;

       // Maybe throw an exception in here instead of exiting? 
       if (userFault) 
       {
           printf("serviceAsynDriver::bsasCallback(): ERROR - Please ensure BSAS channels do not violate 32-bit boundaries!!\n");
           printf("serviceAsynDriver::bsasCallback(): ERROR - Check type for channel %s!!\n", plist->bsas_name); 
           printf("serviceAsynDriver::bsasCallback(): ERROR - Exiting ...\n");
           exit(EXIT_FAILURE);
       }
    } // end of the all activeChannels for-loop
}


extern "C" {


int bsasAssociateBsaChannels(const char *port_name)
{
    return associateBsaChannels(port_name);
}

int bsasBaseName(const char *bsasKey, const char *pv_name)
{
    return bsasPVName(bsasKey, pv_name);
}

int bsasAsynDriverConfigure(const char *portName, const char *reg_path,
                            const char *ntTable_name1,
                            const char *ntTable_name2,
                            const char *ntTable_name3,
                            const char *ntTable_name4,
                            const char *named_root)
{
    pDrvList_t *pl = find_drvByPort(portName);
    if(!pl) {
        pl = find_drvLast();
        if(pl) {
            if(!pl->port_name && !pl->named_root && !pl->pBsasDrv) pl->port_name = epicsStrDup(portName);
            else pl = NULL;
        }
    }

    if(!pl) {
        printf("BSAS list never been configured for port (%s)\n", portName);
        return -1;
    }

    pl->named_root = (named_root && strlen(named_root))? epicsStrDup(named_root): cpswGetRootName();

    if(!pl->pBsasEllList) return -1;

    int i = 0;
    int j = 0;
    int bitSum = 0;
    bsasList_t *p = (bsasList_t *) ellFirst(pl->pBsasEllList);
    while(p) {
        // Number of parameters
        i += (int) (&p->p_lastParam - &p->p_firstParam -1);
        // Assign the bsas/software channel index
        p->swChIndex = j++;
        // Assign the hardware channel index
        bitSum += bsasBitMap[p->type];
        p->hwChIndex = std::floor((bitSum - 1)/BLOCK_WIDTH_32);
        // Advance to the next node
        p = (bsasList_t *) ellNext(&p->node);
    }    /* calculate number of dynamic parameters */

    pl->port_name = epicsStrDup(portName);
    pl->reg_path  = epicsStrDup(reg_path);
    pl->pBsasDrv  = new bsasAsynDriver(portName, reg_path, i, pl->pBsasEllList,
                                       ntTable_name1,
                                       ntTable_name2,
                                       ntTable_name3,
                                       ntTable_name4,
                                       pl->named_root);

    return 0;
}


static const iocshArg initArg0 = {"port name",                                            iocshArgString};
static const iocshArg initArg1 = {"register path (which should be described in yaml): ",  iocshArgString};
static const iocshArg initArg2 = {"NTTable Name 1: ",                                     iocshArgString};
static const iocshArg initArg3 = {"NTTable Name 2: ",                                     iocshArgString};
static const iocshArg initArg4 = {"NTTable Name 3: ",                                     iocshArgString};
static const iocshArg initArg5 = {"NTTable name 4: ",                                     iocshArgString};
static const iocshArg initArg6 = {"named_root (optional)",                                iocshArgString};
static const iocshArg *const initArgs[] = { &initArg0,
                                            &initArg1,
                                            &initArg2, 
                                            &initArg3, 
                                            &initArg4, 
                                            &initArg5, 
                                            &initArg6 };
static const iocshFuncDef initFuncDef = {"bsasAsynDriverConfigure", 7, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    bsasAsynDriverConfigure(args[0].sval,                                                   /* port name */
                            args[1].sval,                                                   /* register path */
                            args[2].sval,                                                   /* NTTable name 1 */
                            args[3].sval,                                                   /* NTTable name 2 */
                            args[4].sval,                                                   /* NTTable name 3 */
                            args[5].sval,                                                   /* NTTable name 4 */
                            (args[6].sval && strlen(args[6].sval))? args[6].sval: NULL);    /* named root */
}

static const iocshArg associateArg0 = {"bsa port", iocshArgString};
static const iocshArg * const associateArgs[] = { &associateArg0 };
static const iocshFuncDef associateFuncDef = {"bsasAssociateBsaChannels", 1, associateArgs};
static void associateCallFunc(const iocshArgBuf *args)
{
    associateBsaChannels(args[0].sval);
}


static const iocshArg baseNameArg0 = {"bsss name",           iocshArgString};
static const iocshArg baseNameArg1 = {"base name (PV name)", iocshArgString};
static const iocshArg *const baseNameArgs[] = { &baseNameArg0,
                                                &baseNameArg1 };
static const iocshFuncDef baseNameFuncDef = {"bsasBaseName", 2, baseNameArgs};
static void baseNameCallFunc(const iocshArgBuf *args)
{
    bsasPVName(args[0].sval, args[1].sval);
}


void bsasAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,      initCallFunc);
    iocshRegister(&associateFuncDef, associateCallFunc);
    iocshRegister(&baseNameFuncDef,  baseNameCallFunc);
}

epicsExportRegistrar(bsasAsynDriverRegister);



/* EPICS driver support for bsasAsynDriver */

static int bsasAsynDriverReport(int interest);
static int bsasAsynDriverInitialize(void);

static struct drvet bsasAsynDriver = {
    2, 
    (DRVSUPFUN) bsasAsynDriverReport,
    (DRVSUPFUN) bsasAsynDriverInitialize
};

epicsExportAddress(drvet, bsasAsynDriver);

static int bsasAsynDriverReport(int interest)
{

    return 0;
}


static int bsasAsynDriverInitialize(void)
{

    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        p->pBsasDrv->lateInit();    // prepare NTTable PVs
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

}    /* extern C */




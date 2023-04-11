#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <tgmath.h>
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
#include <serviceAsynDriver.h>
#include <bsaAsynDriver.h>

#include <yamlLoader.h>

#include <pvxs/server.h>
#include <pvxs/sharedpv.h>
#include <pvxs/log.h>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>

#include "devBsss.h"

#define BSSS_ONLY(TYPE, X) \
if(TYPE == bld) { printf("(%s: %d %s) is only for BSSS\n", __FILE__, __LINE__, __func__); return X; }

#define BLD_ONLY(TYPE, X) \
if(TYPE == bsss) { printf("(%s:%d %s) is only for BLD\n", __FILE__, __LINE__, __func__); return X; }


static const char *driverName = "serviceAsynDriver";


static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *port_name;
    char            *reg_path;
    serviceAsynDriver  *pServiceDrv;
    ELLLIST         *pChannelEllList;
} pDrvList_t;


static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "serviceAsynDriver(init_drvList)");
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
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "serviceAsynDriver(prep_drvAnonimous)");

    p->named_root = NULL;
    p->port_name  = NULL;
    p->reg_path   = NULL;
    p->pServiceDrv   = NULL;
    p->pChannelEllList  = NULL;

    ellAdd(pDrvEllList, &p->node);
    return ellCount(pDrvEllList);
}


static int channelAdd(const char *channelKey, serviceDataType_t type, double *slope, double *offset, bool doNotTouch)
{
    pDrvList_t * pl = find_drvLast();

    if(pl){
        if(pl->pServiceDrv || (pl->port_name && strlen(pl->port_name))) pl = NULL;  /* the driver node has been configured,
                                                                                    need to make another one */
    }

    while(!pl) {
        prep_drvAnonimous();
        pl = find_drvLast();
    }

    while(!pl->pChannelEllList) {   /* initialize the linked list once */
        pl->pChannelEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "serviceAsynDriver (channelAdd)");
        ellInit(pl->pChannelEllList);
    }

    channelList_t *p = (channelList_t *) mallocMustSucceed(sizeof(channelList_t), "serviceAsynDriver (channelAdd)");
    strcpy(p->channel_key, channelKey);
    p->swChIndex = -1;
    p->hwChIndex = -1;
    p->p_channelMask = -1;
    p->p_channelSevr = -1;
    for(unsigned int i = 0; i < NUM_EDEF_MAX; i++) {
        p->p_channel[i] = -1;              /* initialize paramters with invalid */
        p->pkey_channel[i][0] = '\0';     /* initialize with a null string */
        p->pkey_channelPID[i][0] = '\0';  /* initialize with a null string */
        p->pidPv[i].dpvt = NULL;
        p->pidPv[i].pid  = 0;
        p->vPv[i].dpvt = NULL;
        p->vPv[i].v = 0.; 
    }

    p->type    = type;
    p->pslope  = slope;
    p->poffset = offset;
    p->doNotTouch = doNotTouch;

    ellAdd(pl->pChannelEllList, &p->node);
    return 0;
}


static int associateBsaChannels(const char *port_name)
{
    ELLLIST *pBsaEllList = find_bsaChannelList(port_name);

    if(!pBsaEllList) return -1;
    bsaList_t * p = (bsaList_t *) ellFirst(pBsaEllList);

    while(p) {
        channelAdd(p->bsa_name, serviceDataType_t(p->type), &p->slope, &p->offset, p->doNotTouch);
        p = (bsaList_t *) ellNext(&p->node);
    }
    printf("Associate %d of channels from bsa port(%s) \n", ellCount(find_drvLast()->pChannelEllList), port_name);

    return 0;
}

extern "C" {

// Provide associate functions to cexp
int bsssAssociateBsaChannels(const char *port_name)
{ 
    return associateBsaChannels(port_name);
}

int bldAssociateBsaChannels(const char *port_name)
{ 
    return associateBsaChannels(port_name);
}

int bldChannelName(const char *channel_key, const char *channel_name)
{

  /* Implement EPICS driver initialization here */
    init_drvList();

    if(!pDrvEllList) {
        printf("BSSS/BLD Driver never been configured\n");
        return 0;
    }

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->pServiceDrv->getServiceType() == bld) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }    

    /* Found BLD driver. Create PVA */
    if (p != NULL)
        p->pServiceDrv->addBldChannelName(channel_key, channel_name);
    else {
        printf("Error: No BLD driver found. Must instantiate driver first.\n");
        return -1;
    }

    return 0;
}

} // extern "C"


static void bsss_callback(void *pUsr, void *buf, unsigned size)
{
    ((serviceAsynDriver *)pUsr)->bsssCallback(buf, size);
}

static void bld_callback(void *pUsr, void *buf, unsigned size)
{
    ((serviceAsynDriver *)pUsr)->bldCallback(buf, size);
}


serviceAsynDriver::serviceAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pChannelEllList,  serviceType_t type, const char *named_root, const char* pva_basename)
    : asynPortDriver(portName,
                                        1,  /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                                         NUM_SERVICE_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
#endif /* asyn version check, under 4.32 */
                                         asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask |
                                         asynInt32ArrayMask | asynInt64ArrayMask | asynFloat64ArrayMask,    /* Interface mask */
                                         asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask | asynEnumMask |
                                         asynInt32ArrayMask | asynInt64ArrayMask | asynFloat64ArrayMask,       /* Interrupt mask */
                                         1,    /* asynFlags. This driver does block and it is non multi-device, so flag is 1. */
                                         1,    /* Auto connect */
                                         0,    /* Default priority */
                                         0),    /* Default stack size */
                                         channelMask(0)

{
    if(!pChannelEllList || !ellCount(pChannelEllList)) return;   /* if there is no service data channels in the list, nothing to do */
    this->pChannelEllList = pChannelEllList;

    char reg_path0[128], reg_path1[128];
    Path reg_, reg0_, reg1_;
    Path root_ = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
    if(!root_) {
        printf("%s driver: could not find root path\n", type == bsss? "BSSS" : "BLD");
        return;
    }

    switch (type) {
        case bld:
            reg_ = root_->findByName(reg_path);
            if(!reg_) {
                printf("BLD driver: could not find regisers at path %s\n", reg_path);
                return;
            }
            this->pService[0] = new Bld::BldYaml(reg_);
            this->pService[1] = NULL;
            this->numMod = NUM_BLD_MOD;
            break;
        case bsss:
            sprintf(reg_path0, "%s%s", reg_path, "0");
            sprintf(reg_path1, "%s%s", reg_path, "1");
            reg0_ = root_->findByName(reg_path0);
            reg1_ = root_->findByName(reg_path1);
            if(!reg0_ || !reg1_) {
                printf("BSSS driver: could not find register at path %s\n", reg_path);
                return;
            }
            this->pService[0] = new Bsss::BsssYaml(reg0_, BSSS0_NUM_EDEF);
            this->pService[1] = new Bsss::BsssYaml(reg1_, BSSS1_NUM_EDEF);
            this->numMod = NUM_BSSS_MOD;
            break;
    }
    serviceType = type;
    
    channelSevr = 0;

    // Initialize hardware/software channel map
    for (int i = 0; i < HW_CHANNELS; i++)
        hwChannelUsage[i] = {};

    int i = 0;
    int bitSum = 0;
    channelList_t *p = (channelList_t *) ellFirst(pChannelEllList);
    while(p) {
        // First check to see if the current BSA/software channel is less than 32-bits.
        // For the time being, we don't allow BLD to run if any of these is less than 32-bits.
        if (serviceType == bld && channelBitMap[p->type] < BLOCK_WIDTH_32)
        {
            printf("serviceAsynDriver::serviceAsynDriver(): ERROR - The BLD acquisition service is disabled for types less than 32 bits!!\n");
            printf("serviceAsynDriver::serviceAsynDriver(): ERROR - Please use BSA, BSSS or BSAS instead!!\n"); 
            printf("serviceAsynDriver::serviceAsynDriver(): ERROR - Exiting ...\n");
            exit(EXIT_FAILURE);
        }
        // Assign the channel index
        p->swChIndex = i++;
        // Assign the hardware channel index
        bitSum += channelBitMap[p->type];
        p->hwChIndex = std::floor((bitSum - 1)/BLOCK_WIDTH_32);
        hwChannelUsage[p->hwChIndex].push_back(p->swChIndex);
        p = (channelList_t *) ellNext(&p->node);
    }

    SetupAsynParams(type);

    switch (type) {
        case bld:      
            for(unsigned int i = 0; i < this->pService[0]->getEdefNum(); i++)
            {
                socketAPIInitByInterfaceName(ntohl( inet_addr( DEFAULT_MCAST_IP ) ), 
                                    DEFAULT_MCAST_PORT, 
                                    MAX_BUFF_SIZE, 
                                    UCTTL, 
                                    NULL, 
                                    &pVoidsocketAPI[i]);
                if ( pVoidsocketAPI[i] == NULL ) 
                    printf("Failed instantiating new socketAPI for service %u\n", i);
            }
            bldPacketPayload = (uint32_t *) mallocMustSucceed(MAX_BUFF_SIZE, "bldPacketPayload");
        
            pvaBaseName = epicsStrDup(pva_basename);    

            registerBldCallback(named_root, bld_callback, (void *) this); 
            break;
        case bsss:
            // Register BSSS callback
            registerBsssCallback(named_root, bsss_callback, (void *) this); 
            // Set channel mask for BSSS
            for (const auto &kv:hwChannelUsage)
            {
                if (kv.second.size() != 0) // if hardware channel is used, set the mask bit
                {
                    pService[0]->setChannelMask(kv.first,ENABLE);
                    pService[1]->setChannelMask(kv.first,ENABLE);
                }
            }
            break;
    }        
}


void serviceAsynDriver::addBldChannelName(const char * key, const char * name)
{
    bool found = false;
    for (channelList_t *plist = (channelList_t *) ellFirst(pChannelEllList);
        plist!=NULL;
        plist = (channelList_t *) ellNext(&plist->node))
    {
        if (strcmp(plist->channel_key, key) == 0)
        {
            strcpy(plist->channel_name, name);
            found = true;
        }
    }    
    if (found == false)
        printf("Error: BsaKey not found.\n");
}


void serviceAsynDriver::updatePVA()
{
    std::string pvaName(pvaBaseName);
    pvaName = pvaName + ":BLD_PAYLOAD";

    server.removePV(pvaName);
    pv.close();

    this->initPVA();
}

void serviceAsynDriver::initPVA()
{

    channelList_t *plist;
    uint32_t mask;

    pvxs::shared_array<const std::string> _labels;  
    pvxs::TypeDef                         _def;
    pvxs::Value                           _initial;
    char * name;

    std::string pvaName(pvaBaseName);
    pvaName = pvaName + ":BLD_PAYLOAD";

    server = pvxs::ioc::server();

    auto labels = std::vector<std::string>();
    auto value  = pvxs::TypeDef(pvxs::TypeCode::Struct, {});

    pv = pvxs::server::SharedPV::buildReadonly();

    for (plist = (channelList_t *) ellFirst(pChannelEllList), mask = 0x1;
        plist!=NULL;
        plist = (channelList_t *) ellNext(&plist->node), mask <<= 1)
    {
        if ((mask & channelMask) == 0)
            continue;

        if (plist->channel_name == NULL)
            name = plist->channel_key;
        else
            name = plist->channel_name;

        if (plist->doNotTouch == 1)
            value += {pvxs::Member(pvxs::TypeCode::UInt32A, name)};    
        else
            value += {pvxs::Member(pvxs::TypeCode::Float32, name)};    
        labels.push_back(name);        
    }

    _labels = pvxs::shared_array<const std::string>(labels.begin(), labels.end());
    _def = pvxs::TypeDef(pvxs::TypeCode::Struct, "epics:nt/NTScalar:1.0", {
                         value.as("BldPayload")
                         });
    
    _initial              = _def.create();


    pv.open(_initial);
    server.addPV(pvaName, pv);
}

serviceAsynDriver::~serviceAsynDriver()
{
}

void serviceAsynDriver::SetupAsynParams(serviceType_t type)
{
    char param_name[100];
    char prefix[10];
    char prefix2[2][10];

    switch (type) {
        case bld:
            sprintf(prefix, BLD_STR);              // prefix for single instance
            sprintf(prefix2[0], BLD_STR);          // prefix for 1st modue, but actually single instance
            sprintf(prefix2[1], BLD_STR);          // expect never use this
            break;
        case bsss:
            sprintf(prefix, BSSS_STR);             // prefix for single instance
            sprintf(prefix2[0], BSSS0_STR);        // prefix for 1st module
            sprintf(prefix2[1], BSSS1_STR);        // prefix for 2nd module
            break;
    }

    for(int mod = 0; mod < this->numMod; mod++) {  // both BSSS and BLD, BSSS has 2 instances, BLD has 1 instances 
        // Service Status Monitoring
        sprintf(param_name, CURRPACKETSIZE_STR, prefix2[mod]);   createParam(param_name, asynParamInt32, &p_currPacketSize[mod]);
        sprintf(param_name, CURRPACKETSTATUS_STR, prefix2[mod]); createParam(param_name, asynParamInt32, &p_currPacketStatus[mod]);
        sprintf(param_name, CURRPULSEIDL_STR, prefix2[mod]);     createParam(param_name, asynParamInt32, &p_currPulseIdL[mod]);
        sprintf(param_name, CURRTIMESTAMPL_STR, prefix2[mod]);   createParam(param_name, asynParamInt32, &p_currTimeStampL[mod]);
        sprintf(param_name, CURRDELTA_STR, prefix2[mod]);        createParam(param_name, asynParamInt32, &p_currDelta[mod]);
        sprintf(param_name, PACKETCOUNT_STR, prefix2[mod]);      createParam(param_name, asynParamInt32, &p_packetCount[mod]);
        sprintf(param_name, PAUSED_STR, prefix2[mod]);           createParam(param_name, asynParamInt32, &p_paused[mod]);
        sprintf(param_name, DIAGNCLOCKRATE_STR, prefix2[mod]);   createParam(param_name, asynParamInt32, &p_diagnClockRate[mod]);
        sprintf(param_name, DIAGNSTROBERATE_STR, prefix2[mod]);  createParam(param_name, asynParamInt32, &p_diagnStrobeRate[mod]);
        sprintf(param_name, EVENTSEL0RATE_STR, prefix2[mod]);    createParam(param_name, asynParamInt32, &p_eventSel0Rate[mod]);
     }
        // Service Status Control
    sprintf(param_name, PACKETSIZE_STR, prefix);       createParam(param_name, asynParamInt32, &p_packetSize);
    sprintf(param_name, ENABLE_STR, prefix);          createParam(param_name, asynParamInt32, &p_enable);
    

    channelList_t *p = (channelList_t *) ellFirst(pChannelEllList);
    while(p) {
        sprintf(param_name, CHANNELMASK_STR, prefix, p->channel_key); createParam(param_name, asynParamInt32, &(p->p_channelMask)); 
        sprintf(param_name, CHANNELSEVR_STR, prefix, p->channel_key); createParam(param_name, asynParamInt32, &(p->p_channelSevr));
        p = (channelList_t *) ellNext(&p->node);
    }

    // Service Rate Controls
    if(type == bld) {    // rate control is required by BLD only
    for(unsigned int i = 0; i < this->pService[0]->getEdefNum(); i++) {
        sprintf(param_name, EDEFENABLE_STR, prefix, i);createParam(param_name, asynParamInt32, &p_edefEnable[i]);
        sprintf(param_name, RATEMODE_STR, prefix, i);  createParam(param_name, asynParamInt32, &p_rateMode[i]);
        sprintf(param_name, FIXEDRATE_STR, prefix, i); createParam(param_name, asynParamInt32, &p_fixedRate[i]);
        sprintf(param_name, ACRATE_STR, prefix, i);    createParam(param_name, asynParamInt32, &p_acRate[i]);
        sprintf(param_name, TSLOTMASK_STR, prefix, i); createParam(param_name, asynParamInt32, &p_tSlotMask[i]);
        sprintf(param_name, EXPSEQNUM_STR, prefix, i); createParam(param_name, asynParamInt32, &p_expSeqNum[i]);
        sprintf(param_name, EXPSEQBIT_STR, prefix, i); createParam(param_name, asynParamInt32, &p_expSeqBit[i]);
        sprintf(param_name, DESTMODE_STR, prefix, i);  createParam(param_name, asynParamInt32, &p_destMode[i]);
        sprintf(param_name, DESTMASK_STR, prefix, i);  createParam(param_name, asynParamInt32, &p_destMask[i]);
        sprintf(param_name, RATELIMIT_STR, prefix, i); createParam(param_name, asynParamInt32, &p_rateLimit[i]);

        /* BLD multicast port and addresses */
        if (type == bld) {
            sprintf(param_name, BLDMULTICASTADDR_STR, i); createParam(param_name, asynParamOctet, &p_multicastAddr[i]);
            sprintf(param_name, BLDMULTICASTPORT_STR, i); createParam(param_name, asynParamInt32, &p_multicastPort[i]);
        }
    }
    }

    /* PVs only existant in BSSS driver */
    if (type == bsss)
    {
        sprintf(param_name, RATELIMIT_PM_STR, prefix);  createParam(param_name, asynParamInt32, &p_rateLimitBsss);

        // set up dyanamic paramters
        for(unsigned int i = 0; i < (this->pService[0]->getEdefNum() + this->pService[1]->getEdefNum()); i++) {
            channelList_t *p  = (channelList_t *) ellFirst(this->pChannelEllList);
            while(p) {
                sprintf(param_name, BSSSPV_STR,  p->channel_key, i); createParam(param_name, asynParamFloat64, &p->p_channel[i]);    strcpy(p->pkey_channel[i],    param_name);
                sprintf(param_name, BSSSPID_STR, p->channel_key, i); createParam(param_name, asynParamInt64,   &p->p_channelPID[i]); strcpy(p->pkey_channelPID[i], param_name);
                p = (channelList_t *) ellNext(&p->node);
            }
        }
    }

}


void serviceAsynDriver::SetRate(int chn)
{
    BLD_ONLY(serviceType,);

    uint32_t rateMode, fixedRate, acRate, tSlotMask, expSeqNum, expSeqBit;

    getIntegerParam(p_rateMode[chn], (epicsInt32*) &rateMode);
    switch(rateMode) {
        case 0: /* fixed rate mode */
            getIntegerParam(p_fixedRate[chn], (epicsInt32*) &fixedRate);
            pService[0]->setFixedRate(chn, fixedRate);
            break;
        case 1: /* AC rate mode */
            getIntegerParam(p_acRate[chn], (epicsInt32*) &acRate);
            getIntegerParam(p_tSlotMask[chn], (epicsInt32*) &tSlotMask);
            pService[0]->setACRate(chn, tSlotMask, acRate);
            break;
        case 2: /* Seq rate mode */
            getIntegerParam(p_expSeqNum[chn], (epicsInt32*) &expSeqNum);
            getIntegerParam(p_expSeqBit[chn], (epicsInt32*) &expSeqBit);
            pService[0]->setSeqRate(chn, expSeqNum, expSeqBit);
            break;
        default:  /* nothing todo */
            break;
    }
}

void serviceAsynDriver::SetDest(int chn)
{
    BLD_ONLY(serviceType,);

    uint32_t destMode, destMask;

    getIntegerParam(p_destMode[chn], (epicsInt32*) &destMode);
    getIntegerParam(p_destMask[chn], (epicsInt32*) &destMask);

    switch(destMode) {
        case 2:  /* Inclusion */
            pService[0]->setDestInclusion(chn, destMask);
            break;
        case 1:  /* Exclusion */
            pService[0]->setDestExclusion(chn, destMask);
            break;
        case 0:  /* Disable */
            pService[0]->setDestDisable(chn);
            break;
        default:  /* nothing to do */
            break;
    }
}

void serviceAsynDriver::SetChannelSevr(int chn, int sevr)
{
    uint64_t mask = 0x3 << (chn*2);

    channelSevr &= ~mask;
    channelSevr |= (uint64_t(sevr) << (chn*2)) & mask;

}

int serviceAsynDriver::GetChannelSevr(int chn)
{
   return int((channelSevr >> (chn *2)) & 0x3);
}

void serviceAsynDriver::MonitorStatus(void)
{
    uint32_t v;

    for(int i = 0; i < this->numMod; i++) {
        pService[i]->getCurrPacketSize(&v);    setIntegerParam(p_currPacketSize[i], (epicsInt32) v);
        pService[i]->getCurrPacketState(&v);   setIntegerParam(p_currPacketStatus[i], (epicsInt32) v);
        pService[i]->getCurrPulseIdL(&v);      setIntegerParam(p_currPulseIdL[i], (epicsInt32) v);
        pService[i]->getCurrTimeStampL(&v);    setIntegerParam(p_currTimeStampL[i], (epicsInt32) v);
        pService[i]->getCurrDelta(&v);         setIntegerParam(p_currDelta[i], (epicsInt32) v);
        pService[i]->getPacketCount(&v);       setIntegerParam(p_packetCount[i], (epicsInt32) v);
        pService[i]->getPaused(&v);            setIntegerParam(p_paused[i], (epicsInt32) v);
        pService[i]->getDiagnClockRate(&v);    setIntegerParam(p_diagnClockRate[i], (epicsInt32) v);
        pService[i]->getDiagnStrobeRate(&v);   setIntegerParam(p_diagnStrobeRate[i], (epicsInt32) v);
        pService[i]->getEventSel0Rate(&v);     setIntegerParam(p_eventSel0Rate[i], (epicsInt32) v);
    }
    callParamCallbacks();
}


asynStatus serviceAsynDriver::writeOctet (asynUser *pasynUser, const char *value,
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";

    if(serviceType == bld) {
    for(unsigned int edef = 0; edef < this->pService[0]->getEdefNum(); edef++) {
        if(function == p_multicastAddr[edef]) {
            socketAPISetAddr(ntohl( inet_addr( value )), pVoidsocketAPI[edef]);
            break;
        }
    }
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: status=%d, function=%d, value=%s",
                  driverName, functionName, status, function, value);
    else
    {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%s\n",
              driverName, functionName, function, value);
        callParamCallbacks();
    }
    *nActual = nChars;
    return status;
}

asynStatus serviceAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";

    /* set the parameter in the parameter library */
    status = (asynStatus) setIntegerParam(function, value);

    channelList_t *p = (channelList_t *) ellFirst(pChannelEllList);
    while(p) 
    {
        if(function == p->p_channelMask) 
        {
            // If the hardware channel is being disabled, 
            // check first if it is used by other software channels.
            // If it is used, then ignore the user request to disable
            //
            if (serviceType == bld) // handling channel mask for BLD
            {    
                if (value == 0)
                    channelMask = channelMask & ~(0x1U << p->hwChIndex);
                else
                    channelMask = channelMask | (0x1U << p->hwChIndex);
                pService[0]->setChannelMask(p->hwChIndex, uint32_t(value));
                updatePVA();
            } 
            else // handling channel mask for BSSS 
            {    
                if (value) // enable hardware channel
                {
                    // Search for software channel in the channel map
                    auto vec = hwChannelUsage[p->hwChIndex];
                    if (std::find(vec.begin(),vec.end(),p->swChIndex) == vec.end())
                    {
                        // Only set the hardware channel bit if not set before
                        if (hwChannelUsage[p->hwChIndex].size() == 0)
                        {
                            pService[0]->setChannelMask(p->hwChIndex, uint32_t(value));
                            pService[1]->setChannelMask(p->hwChIndex, uint32_t(value));
                        }
                    
                        // Add software channel to the usage list
                        hwChannelUsage[p->hwChIndex].push_back(p->swChIndex);
                    }
                }
                else // disable hardware channel
                {
                    // Search for software channel in the channel map
                    auto pos = std::find(hwChannelUsage[p->hwChIndex].begin(),hwChannelUsage[p->hwChIndex].end(),p->swChIndex);
                    if (pos != hwChannelUsage[p->hwChIndex].end())
                    {
                        // Remove software channel from the usage list
                        hwChannelUsage[p->hwChIndex].erase(pos);
                        
                        // Disable hardware channel only if not used by any other software channels
                        if (hwChannelUsage[p->hwChIndex].size() == 0)
                        {
                            pService[0]->setChannelMask(p->hwChIndex, uint32_t(value));
                            pService[1]->setChannelMask(p->hwChIndex, uint32_t(value));
                        }
                    }
                }
           } 
           goto done;
       }
 
       if(function == p->p_channelSevr) {
           SetChannelSevr(p->hwChIndex, value);
           goto done;
       }
        p = (channelList_t *) ellNext(&p->node);
    }


    if(function == p_packetSize) {
        pService[0]->setPacketSize((uint32_t) value);                          // commmon, both for bld and bsss
        if(serviceType == bsss) pService[1]->setPacketSize((uint32_t) value);  // only for bsss
        goto done;
    }
    else if(function == p_enable) {
        pService[0]->enablePacket((uint32_t) value);                            // common, both for bld and bsss
        if(serviceType == bsss) pService[1]->enablePacket((uint32_t) value);    // only for bsss
        goto done;
    }


    if(serviceType == bld) {     // EDEF rate control for bld only
    for(unsigned int i = 0; i < this->pService[0]->getEdefNum(); i++) {
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
            pService[0]->setEdefEnable(i, (uint32_t) value);
            goto done;
        }
        else if(function == p_rateLimit[i]) {
            pService[0]->setRateLimit(i, (uint32_t) value);
            goto done;
        } else if(function == p_multicastPort[i]) {
            socketAPISetPort( value, pVoidsocketAPI[i]);
            goto done;
        }
    }
    }

    if(serviceType == bsss && function == p_rateLimitBsss) {
        pService[0]->setRateLimit((uint32_t) value);
        pService[1]->setRateLimit((uint32_t) value);
        goto done;
    }

    done:
    callParamCallbacks();
    return status;
}

void serviceAsynDriver::printMap()
{
    for(int i=0;i<HW_CHANNELS;i++)
    {
        printf("| %d | -> ",i);
        for(auto x:hwChannelUsage[i])
        {
            printf("%d, ",x);
        }
        printf("\n");
    }
}

void serviceAsynDriver::llrfPerformChecks(const channelList_t * pv, const channelList_t * pvN, int bitIndex)
{
    // Check if channel pointers are nullptr
    if (pv == nullptr || pvN == nullptr)
    {
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - LLRF BSA channels are nullptr!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Check LLRF BSA channels!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Exiting ...\n");
        exit(EXIT_FAILURE);
    }
    // Ensure bit index points to the start of the channel
    if (bitIndex != 0)
    {
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - LLRF BSA channel not aligned with LSB!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Check LLRF BSA channel order!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Exiting ...\n");
        exit(EXIT_FAILURE);
    }
    // Ensure the types of consecutive channels are valid (phase->amp or amp->phase)
    if ((pv->type == llrfAmp_service   && pvN->type == llrfPhase_service) ||
        (pv->type == llrfPhase_service && pvN->type == llrfAmp_service  )   ){}
    else
    {
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Did NOT find consecutive Amplitude and Phase channels!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Check LLRF BSA channel types!!\n");
        printf("serviceAsynDriver::llrfPerformChecks(): ERROR - Exiting ...\n");
        exit(EXIT_FAILURE);
    }
}

void serviceAsynDriver::llrfCalcPhaseAmp(signed short i, signed short q, double& amp, double& phase)
{
    // Calculate amplitude 
    amp = (!isnan(i) && !isnan(q))?sqrt(i * i + q * q):0.0;
    // Calculate phase
    phase = (!isnan(i) && !isnan(q) && i != 0)?atan2((double)q, (double)i) * M_PI_DEGREES / M_PI:0.0;
}

void serviceAsynDriver::bsssCallback(void *p, unsigned size)
{
     uint32_t *buf         = (uint32_t *) p;
     uint32_t *p_uint32    = buf + IDX_DATA;
     int32_t  *p_int32     = (int32_t *) (buf + IDX_DATA);
     float    *p_float32   = (float*)(buf + IDX_DATA);
     double   val;
     uint32_t channel_mask = buf[IDX_CHN_MASK];
     uint32_t service_mask = buf[IDX_SVC_MASK];
     uint64_t sevr_mask    = *(uint64_t*) (buf+IDX_SEVR_MASK(size));
     uint64_t pulse_id     = ((uint64_t)(buf[IDX_PIDU])) << 32 | buf[IDX_PIDL];

     uint32_t tmpVal, mask, iVal, qVal;
     double amp, phase, quant1, quant2;

     int mod = (service_mask & ((uint32_t) 0x1 << BSSS1_BIT))?1:0;   // decide BSSS0 or BSSS1

     epicsTimeStamp _ts;
     _ts.nsec = buf[IDX_NSEC];
     _ts.secPastEpoch  = buf[IDX_SEC];
     // setTimeStamp(&_ts);    // timestamp update for asyn port and related PVs

     // Loop over data channels (i.e. PVs)
     channelList_t *plist = (channelList_t *) ellFirst(pChannelEllList);

     int hwChIndex = 0;        // data/firmware channel number
     int dataIndex = 0;        // indicate the data location

     // Variable to keep track of bit boundaries as we read in channel data
     unsigned bitSum = 0;

     while(plist) {
         // Skipping the (firmware) channel, if the channel is not enabled in the mask
         while(!(channel_mask & (uint32_t(0x1) << hwChIndex)))
         {
             hwChIndex++; // evolve data channel number to next channel
             if (hwChIndex >= BLOCK_WIDTH_32)
                 return;
         }
         // Get a pointer to the next channel
         channelList_t *plistN = (channelList_t *) ellNext(&plist->node);

         // Count newly extracted bits only once below, regardless of multiple EDEFs
         unsigned newBitsExtracted;

         // Now loop over all EDEFs
         for (unsigned int i = 0, svc_mask = 0x1; i < this->pService[mod]->getEdefNum(); i++, svc_mask <<= 1) {
             int edef = i + mod * BSSS0_NUM_EDEF;    // calculate edef number
             if(service_mask & svc_mask) {
                 plist->pidPv[edef].pid = pulse_id;
                 plist->pidPv[edef].time = _ts;
                 process_pidPv(&plist->pidPv[edef]);

                 if(int((sevr_mask >> (hwChIndex*2)) & 0x3) <= GetChannelSevr(hwChIndex) ) {  // data update for valid mask
                     switch(plist->type){
                         case uint2_service:
                             // Read channel data
                             tmpVal = p_uint32[dataIndex];
                             // Bit-shift, extract with mask and assign result to val
                             mask = KEEP_LSB_2; tmpVal >>= bitSum; tmpVal &= mask;
                             val  = (double) (tmpVal);
                             // Increment bitSum counter once and only after done with all EDEFs  
                             newBitsExtracted = BLOCK_WIDTH_2;
                             break;
                         case uint16_service:
                             // Read channel data
                             tmpVal = p_uint32[dataIndex];
                             // Bit-shift, extract with mask and assign result to val
                             mask = KEEP_LSB_16; tmpVal >>= bitSum; tmpVal &= mask;
                             val  = (double) (tmpVal);
                             // Increment bitSum counter once and only after done with all EDEFs  
                             newBitsExtracted = BLOCK_WIDTH_16;
                             break;
                         case llrfAmp_service:
                         case llrfPhase_service:
                             // Perform checks to ensure type validity
                             llrfPerformChecks(const_cast<channelList_t *>(plist),const_cast<channelList_t *>(plistN),bitSum);
                             // Extract lower 16 bits
                             iVal = p_uint32[dataIndex];
                             mask = KEEP_LSB_16;iVal >>= bitSum;iVal &= mask;
                             // Extract upper 16 bits
                             qVal = p_uint32[dataIndex];
                             mask = KEEP_LSB_16;qVal >>= BLOCK_WIDTH_16;qVal &= mask;
                             // Compute phase & amplitude
                             llrfCalcPhaseAmp(static_cast<signed short>(iVal),static_cast<signed short>(qVal), amp, phase);                
                             // Figure out the PV assignment order of the computed values
                             quant1 = (plist->type == llrfAmp_service)?amp:phase;
                             quant2 = (quant1 == amp)?phase:amp;
                             // Assign quant1
                             if(!isnan(quant1)) quant1 = quant1 * (*plist->pslope) + (*plist->poffset);
                             plist->vPv[edef].v = quant1;
                             plist->vPv[edef].time = _ts;
                             process_vPv(&plist->vPv[edef]);
                             // Assign quant2
                             if(!isnan(quant2)) quant2 = quant2 * (*plistN->pslope) + (*plistN->poffset);
                             plistN->vPv[edef].v = quant2;
                             plistN->vPv[edef].time = _ts;
                             process_vPv(&plistN->vPv[edef]);
                             // Increment bitSum counter once and only after done with all EDEFs
                             newBitsExtracted = 2 * BLOCK_WIDTH_16;
                             break;
                         case int32_service:
                             // Read channel data
                             val = (double) (p_int32[dataIndex]);
                             // Increment bitSum counter once and only after done with all EDEFs  
                             newBitsExtracted = BLOCK_WIDTH_32;
                             break;
                         case uint32_service:
                             // Read channel data
                             val = (double) (p_uint32[dataIndex]);
                             // Increment bitSum counter once and only after done with all EDEFs  
                             newBitsExtracted = BLOCK_WIDTH_32;
                             break;
                         case float32_service:
                             // Read channel data
                             val = (double) (p_float32[dataIndex]);
                             // Increment bitSum counter once and only after done with all EDEFs  
                             newBitsExtracted = BLOCK_WIDTH_32;
                             break;
                         case uint64_service:
                         default:
                             val = NAN;   // uint64 never defined
                             newBitsExtracted = 0;
                             break;
                     }
                 } else val = NAN;  // put NAN for invalid mask
                
                 // Assign the value
                 if (plist->type != llrfAmp_service && plist->type != llrfPhase_service){
                     if(!isnan(val)) val = val * (*plist->pslope) + (*plist->poffset);
                     plist->vPv[edef].v = val;
                     plist->vPv[edef].time = _ts;
                     process_vPv(&plist->vPv[edef]);
                 }
             } // end of (service_mask & svc_mask)
         } // end of for-loop for all EDEFs

         // Increment channel pointer if we have an LLRF channel type
         if (plist->type == llrfAmp_service || plist->type == llrfPhase_service)
             plist = (channelList_t *) ellNext(&plist->node);
         
         // Update bitSum counter
         bitSum += newBitsExtracted;

         // Increment data index only if done reading current (firmware) channel 
         // (i.e. may take a few repetitions if reading <32-bit blocks at a time) 
         if (bitSum == BLOCK_WIDTH_32)
         {
             // All good, move on to the next 32-bit word
             bitSum = 0;      // reset bitSum counter
             ++dataIndex;     // point to next 32-bit word in memory buffer
             ++hwChIndex;     // evolve data channel number to next firmware channel
         }
         else if (bitSum > BLOCK_WIDTH_32)
         {
             printf("serviceAsynDriver::bsssCallback(): ERROR - Please ensure BSSS channels do not violate 32-bit boundaries!!\n");
             printf("serviceAsynDriver::bsssCallback(): ERROR - Check type for channel %s\n", plist->channel_name); 
             printf("serviceAsynDriver::bsssCallback(): ERROR - Exiting ...\n");
             exit(EXIT_FAILURE);
         }

         // Evolve to next BSA channel/PV
         plist = (channelList_t *) ellNext(&plist->node);
     } // end of while(pList)
}

serviceType_t serviceAsynDriver::getServiceType()
{
    return serviceType;
}
void serviceAsynDriver::bldCallback(void *p, unsigned size)
{
    
    uint32_t *buf         = (uint32_t *) p;
    uint32_t *p_uint32    = buf + IDX_DATA;
    int32_t  *p_int32     = (int32_t *) (buf + IDX_DATA);
    float    *p_float32   = (float*)(buf + IDX_DATA);
    float    *bldPacketPayloadfloat = (float    *) bldPacketPayload;
    bldAxiStreamHeader_t *header = (bldAxiStreamHeader_t *) p;
    bldAxiStreamComplementaryHeader_t * compHeader;
    double   val;
    int data_chn, dataIndex;
    uint64_t sevr_mask;
    channelList_t *plist;
    static uint32_t versionSize = 0;
    uint32_t multicastIndex = MULTICAST_IDX_DATA;
    uint32_t severityMaskAddrL = MULTICAST_IDX_SEVRL;
    uint32_t severityMaskAddrH = MULTICAST_IDX_SEVRH;
    static uint32_t channelMask = 0;

    /* Matt: versionSize increments whenever channel mask changes */
    if (channelMask != header->channelMask)
    {
        channelMask = header->channelMask;
        versionSize++;
    }
    
    bldPacketPayload[IDX_NSEC] = buf[IDX_NSEC];
    bldPacketPayload[IDX_SEC]  = buf[IDX_SEC];
    bldPacketPayload[IDX_PIDL] = buf[IDX_PIDL];
    bldPacketPayload[IDX_PIDU] = buf[IDX_PIDU];
    bldPacketPayload[IDX_VERSION] = versionSize;
    
    uint32_t consumedSize = sizeof(bldAxiStreamHeader_t);
    do{
    
        for (plist = (channelList_t *) ellFirst(pChannelEllList), dataIndex = 0, data_chn = 0;
            plist!=NULL;
            plist = (channelList_t *) ellNext(&plist->node), data_chn++) 
        {
            if(!(header->channelMask & (uint32_t(0x1) << data_chn)))
                continue;   // skipping the channel, if the channel is not in the mask

            switch(plist->type){
                case int32_service:
                    val = (double) (p_int32[dataIndex]);
                    break;
                case uint32_service:
                    val = (double) (p_uint32[dataIndex]);
                    break;
                case float32_service:
                    val = (double) (p_float32[dataIndex]);
                    break;
                case uint64_service:
                default:
                    val = NAN;   // uint64 never defined
                    break;
            }
            
            if(!isnan(val)) 
                val = val * (*plist->pslope) + (*plist->poffset);

            /* If fixed, apply correct format */
            if (plist->doNotTouch == 1)
                bldPacketPayload[multicastIndex++] = p_uint32[dataIndex]; /* formatted to int32/uint32 (untouched) */
            else
                bldPacketPayloadfloat[multicastIndex++] = val; /* formatted to IEEE 754 */

            dataIndex++; /* Increment dataIndex only if not skipping */
            consumedSize += 4;
        }

        sevr_mask    = *(uint64_t*) (buf + dataIndex + IDX_DATA);
        bldPacketPayload[severityMaskAddrL] = *(buf + dataIndex + IDX_DATA);
        bldPacketPayload[severityMaskAddrH] = *(buf + dataIndex + IDX_DATA + 1);

        consumedSize += sizeof(sevr_mask);

        if (consumedSize >= size)
            break;

        /* If reaches here is because there is more event data */
        compHeader = (bldAxiStreamComplementaryHeader_t *) (buf + (consumedSize/4));

        /* Copy deltas to multicast packet */
        bldPacketPayload[multicastIndex++] = compHeader->deltas_u.deltasCombined;

        /* Store Severity Mask address location */
        severityMaskAddrL = multicastIndex++;
        severityMaskAddrH = multicastIndex++;

        /* Override data type pointers */
        p_uint32    = (uint32_t *) (compHeader + sizeof(bldMulticastPacketComplementaryHeader_t));
        p_int32     = (int32_t *) (compHeader + sizeof(bldMulticastPacketComplementaryHeader_t));
        p_float32   = (float*) (compHeader + sizeof(bldMulticastPacketComplementaryHeader_t));

        consumedSize += sizeof(bldAxiStreamComplementaryHeader_t);
    } while (consumedSize < size);

    for (uint32_t mask = 0xF & (header->serviceMask), it = 0; mask != 0x0; mask >>= 1, it++)
    {
        if ( (mask & 0x1) != 0 )
        {
                socketAPISendRawData(pVoidsocketAPI[it], 
                                                    multicastIndex*sizeof(int), 
                                                    (char*) bldPacketPayload);
        }
    }
}

extern "C" {

/* devBSSS API */
pid_pvt *find_pidPv(char *port, char *key, int edef)
{
    pDrvList_t *pDrv = find_drvByPort(port);
    if(!pDrv || !pDrv->pChannelEllList || ellCount(pDrv->pChannelEllList)==0) return NULL;

    channelList_t * p = (channelList_t *) ellFirst(pDrv->pChannelEllList);
    while(p) {
        if(!strcmp(key, p->channel_key)) break;
        p = (channelList_t *) ellNext(&p->node);
    }

    if(p) return &p->pidPv[edef];
    else  return NULL;
}

v_pvt * find_vPv(char *port, char *key, int edef)
{
    pDrvList_t *pDrv = find_drvByPort(port);
    if(!pDrv || !pDrv->pChannelEllList || ellCount(pDrv->pChannelEllList)==0) return NULL;

    channelList_t * p = (channelList_t *) ellFirst(pDrv->pChannelEllList);
    while(p) {
        if(!strcmp(key, p->channel_key)) break;
        p = (channelList_t *) ellNext(&p->node);
    }

    if(p) return &p->vPv[edef];
    else  return NULL;
}


/* end of devBSSS API */

static int  serviceMonitorPoll(void)
{
    while(1) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pServiceDrv) p->pServiceDrv->MonitorStatus();
            p = (pDrvList_t *) ellNext(&p->node);
        }
        epicsThreadSleep(1.);
    }

    return 0;
}


int serviceAsynDriverConfigure(const char *portName, const char *reg_path, const char *named_root, serviceType_t type, const char* pva_basename)
{
    pDrvList_t *pl = find_drvByPort(portName);
    if(!pl) {
        pl = find_drvLast();
        if(pl) {
            if(!pl->port_name && !pl->named_root && !pl->pServiceDrv) pl->port_name = epicsStrDup(portName);
            else pl = NULL;
        }
    }
    if(!pl) {
        printf("%s list never been configured for port (%s)\n", type == bsss? "BSSS":"BLD", portName);
        return -1;
    }

    pl->named_root = (named_root && strlen(named_root))?epicsStrDup(named_root): cpswGetRootName();

    if(!pl->pChannelEllList) return -1;

    int i = 0;
    channelList_t *p = (channelList_t *) ellFirst(pl->pChannelEllList);
    while(p) {
        i += (int) (&p->p_lastParam - &p->p_firstParam -1);
        p = (channelList_t *) ellNext(&p->node);

    }    /* calculate number of dyanmic parameters */
    if (type == bld && pva_basename == NULL)
    {
        printf("BLD driver pva_basename is NULL. Please add basename.\n");
        return -1;
    }

    pl->port_name = epicsStrDup(portName);
    pl->reg_path  = epicsStrDup(reg_path);
    pl->pServiceDrv  = new serviceAsynDriver(portName, reg_path, i, pl->pChannelEllList, type, pl->named_root, pva_basename);

    return 0;
}

extern "C" {

// Add functions to cexp

int bsssAsynDriverConfigure(const char *portName, const char *reg_path, const char *named_root)
{
    return serviceAsynDriverConfigure( portName,
                                reg_path,
                                named_root && strlen(named_root)? named_root : NULL,
                                bsss,
                                NULL );  /* BSSS call */
}

int bldAsynDriverConfigure(const char *portName, const char *reg_path, const char* pva_basename, const char *named_root)
{
    return serviceAsynDriverConfigure( portName,
                                reg_path,
                                named_root && strlen(named_root)? named_root : NULL,
                                bld,
                                pva_basename );  /* BLD call */
}

} // extern "C"

static const iocshArg initArg0 = { "portName",                                           iocshArgString };
static const iocshArg initArg1 = { "register path (which should be described in yaml):", iocshArgString };
static const iocshArg initArg2 = { "named_root (optional)",                              iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef bsssInitFuncDef = { "bsssAsynDriverConfigure", 3, initArgs };
static void bsssInitCallFunc(const iocshArgBuf *args)
{
 
    serviceAsynDriverConfigure(args[0].sval,  /* port name */
                               args[1].sval,  /* register path */
                              (args[2].sval && strlen(args[2].sval))? args[2].sval: NULL, /* named_root */
                               bsss,
                               NULL );  /* BSSS call */
}

static const iocshArg initBldArg0 = { "portName",                                           iocshArgString };
static const iocshArg initBldArg1 = { "register path (which should be described in yaml)",  iocshArgString };
static const iocshArg initBldArg2 = { "Payload PVA basename",                               iocshArgString };
static const iocshArg initBldArg3 = { "named_root (optional)",                              iocshArgString };
static const iocshArg * const initBldArgs[] = { &initBldArg0,
                                                &initBldArg1,
                                                &initBldArg2,
                                                &initBldArg3 };
static const iocshFuncDef bldInitFuncDef = { "bldAsynDriverConfigure", 4, initBldArgs };
static void bldInitCallFunc(const iocshArgBuf *args)
{
 
    serviceAsynDriverConfigure(args[0].sval,  /* port name */
                               args[1].sval,  /* register path */
                               (args[3].sval && strlen(args[3].sval))? args[3].sval: NULL, /* named_root */
                               bld,
                               args[2].sval  /* bld PVA basename */
                               );  /* BLD call */ 
}


static const iocshArg associateArg0 = { "bsa port", iocshArgString };
static const iocshArg * const associateArgs [] = { &associateArg0 };
static const iocshFuncDef bsssAssociateFuncDef = { "bsssAssociateBsaChannels", 1, associateArgs };
static const iocshFuncDef bldAssociateFuncDef =  { "bldAssociateBsaChannels", 1, associateArgs };

static void associateCallFunc(const iocshArgBuf *args)
{
    associateBsaChannels(args[0].sval);
}

static const iocshArg bldChannelNameArg0 = { "bsaKey",      iocshArgString };
static const iocshArg bldChannelNameArg1 = { "channelName", iocshArgString };
static const iocshArg * const bldChannelNameArgs [] = { &bldChannelNameArg0,
                                                        &bldChannelNameArg1};
static const iocshFuncDef bldChannelNameFuncDef = { "bldChannelName", 2, bldChannelNameArgs };
static void bldChannelNameCallFunc(const iocshArgBuf *args)
{
    bldChannelName(args[0].sval, args[1].sval);
}

void serviceAsynDriverRegister(void)
{
    iocshRegister(&bsssInitFuncDef,        bsssInitCallFunc);
    iocshRegister(&bldInitFuncDef,         bldInitCallFunc);
    iocshRegister(&bsssAssociateFuncDef,   associateCallFunc);
    iocshRegister(&bldAssociateFuncDef,    associateCallFunc);
    iocshRegister(&bldChannelNameFuncDef,  bldChannelNameCallFunc);    
}

epicsExportRegistrar(serviceAsynDriverRegister);




/* EPICS driver support for serviceAsynDriver */

static int serviceAsynDriverReport(int interest);
static int serviceAsynDriverInitialize(void);

static struct drvet serviceAsynDriver = {
    2,
    (DRVSUPFUN) serviceAsynDriverReport,
    (DRVSUPFUN) serviceAsynDriverInitialize
};

epicsExportAddress(drvet, serviceAsynDriver);

static int serviceAsynDriverReport(int interest)
{
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        printf("named_root: %s, port: %s, driver instace: %p, number of channels: %d\n",
              (p->named_root && strlen(p->named_root))?p->named_root: "Unknown",
              (p->port_name && strlen(p->port_name))? p->port_name: "Unknown",
              p->pServiceDrv,
              (p->pChannelEllList)? ellCount(p->pChannelEllList): -1);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int serviceAsynDriverInitialize(void)
{

   /* Implement EPICS driver initialization here */
    init_drvList();

    if(!pDrvEllList) {
        printf("BSSS/BLD Driver never been configured\n");
        return 0;
    }

    printf("BSSS/BLD driver: %d of service driver instance(s) has (have) been configured\n", ellCount(pDrvEllList));

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    epicsThreadCreate("serviceDrvPoll", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) serviceMonitorPoll, 0);

    while(p) {
        if(p->pServiceDrv->getServiceType() == bld) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }    

    /* Found BLD driver. Create PVA */
    if (p != NULL)
        p->pServiceDrv->initPVA();

    return 0;
}


} /* extern "C" */

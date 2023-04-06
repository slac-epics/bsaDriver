#ifndef BSSS_ASYN_DRIVER_H
#define BSSS_ASYN_DRIVER_H

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <new>
#include <arpa/inet.h>
#include <sstream>
#include <map>

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <ellLib.h>

#include <BsssYaml.hh>
#include <BldYaml.hh>

#include <socketAPI.h>

#include <pvxs/server.h>
#include <pvxs/sharedpv.h>
#include <pvxs/log.h>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>

#include "devBsss.h"

#define MAX_MOD_NUM          2   // BSSS has 2 moduels, BLD has 1 moduel now.
#define NUM_BLD_MOD          1   // number of BLD module
#define NUM_BSSS_MOD         2   // number of BSSS modules

#define BSSS1_BIT            28   // indicator for BSSS1
#define BSSS0_NUM_EDEF       28   // BSSS0 module covers 28 EDEFs
#define BSSS1_NUM_EDEF       16   // BSSS1 module covers 16 EDEFs

#define HW_CHANNELS          31
#define NUM_EDEF_MAX         64
#define MAX_BUFF_SIZE        9000
#define CHNMASK_INVALID      0xFFFFFFFF

#define UCTTL                32 /// minimum: 1 + (# of routers in the middle)
#define DEFAULT_MCAST_IP     "239.255.24.1"
#define DEFAULT_MCAST_PORT   50000

#define IDX_NSEC 0
#define IDX_SEC  1
#define IDX_PIDL 2
#define IDX_PIDU 3
#define IDX_VERSION 4
#define IDX_CHN_MASK 4
#define IDX_SVC_MASK 5
#define IDX_DATA 6
#define IDX_SEVR_MASK(size)  (size/4 -2)

#define STR_SIZE 64

#define MULTICAST_IDX_SEVRL 5
#define MULTICAST_IDX_SEVRH 6
#define MULTICAST_IDX_DATA 7

#define FIXED_SLOPE  1
#define FIXED_OFFSET 0

#define ENABLE  1
#define DISABLE 0

#define BLOCK_WIDTH_2   2
#define BLOCK_WIDTH_16  16
#define BLOCK_WIDTH_32  32
#define BLOCK_WIDTH_64  32

#define KEEP_LSB_2    0x00000003 
#define KEEP_LSB_16   0x0000ffff 
#define DEFAULT_MASK  0xffffffff

#define M_PI_DEGREES 180.0

typedef enum {
    uint2_service,
    uint16_service,
    int32_service,
    uint32_service,
    uint64_service,
    float32_service,
    fault_service,
    llrfAmp_service,
    llrfPhase_service
} serviceDataType_t;

static std::map<serviceDataType_t, int> channelBitMap = {{uint2_service,     BLOCK_WIDTH_2 },
                                                         {uint16_service,    BLOCK_WIDTH_16},
                                                         {int32_service,     BLOCK_WIDTH_32},
                                                         {uint32_service,    BLOCK_WIDTH_32},
                                                         {uint64_service,    BLOCK_WIDTH_64},
                                                         {float32_service,   BLOCK_WIDTH_32},
                                                         {llrfAmp_service,   BLOCK_WIDTH_16},
                                                         {llrfPhase_service, BLOCK_WIDTH_16}};

static std::map<int, std::vector<int>> hwChannelUsage;

typedef enum {
    bld,
    bsss
} serviceType_t;

typedef struct {
    ELLNODE node;
    char    channel_key[STR_SIZE];
    char    channel_name[STR_SIZE];

    int     hwChIndex;
    int     swChIndex;

    int     p_firstParam;
    int     p_channelMask;
    int     p_channelSevr;
    int     p_channel[NUM_EDEF_MAX];
    int     p_channelPID[NUM_EDEF_MAX];
    int     p_lastParam;


    serviceDataType_t type;
    double  *pslope;
    double  *poffset;

    /* BSSS only*/
    pid_pvt pidPv[NUM_EDEF_MAX];      // pv for pid, bridge to the devBSSS
    v_pvt   vPv[NUM_EDEF_MAX];        // pv for value, bridge to the devBSSS
    char    pkey_channel[NUM_EDEF_MAX][STR_SIZE];
    char    pkey_channelPID[NUM_EDEF_MAX][STR_SIZE];

    bool    doNotTouch;
} channelList_t;

typedef struct __attribute__((__packed__)) {
    uint64_t timeStamp;
    uint64_t pulseID;
    uint32_t channelMask;
    uint32_t serviceMask;
} bldAxiStreamHeader_t;

typedef struct __attribute__((__packed__)) {
    union deltas_union {
        struct {
            uint32_t deltaTimeStamp:20;
            uint32_t deltaPulseID:12;
        } deltas_struct;
        uint32_t deltasCombined;
    } deltas_u;
    uint32_t serviceMask;
} bldAxiStreamComplementaryHeader_t;

typedef struct __attribute__((__packed__)) {
    uint64_t timeStamp;
    uint64_t pulseID;
    uint32_t version;
    uint64_t severityMask;
} bldMulticastPacketHeader_t;

typedef struct __attribute__((__packed__)) {
    union deltas_union {
        struct {
            uint32_t deltaTimeStamp:20;
            uint32_t deltaPulseID:12;
        } deltas_struct;
        uint32_t deltasCombined;
    } deltas_u;
    uint64_t severityMask;
} bldMulticastPacketComplementaryHeader_t;

typedef struct {

} bldClientConf_t;

class serviceAsynDriver: asynPortDriver {

    public:
        serviceAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pChannelEllList,  serviceType_t type, const char *named_root, const char* pva_basename);
        ~serviceAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
        pvxs::server::SharedPV pvaTest();
        void updatePVA();
        void initPVA();
        void addBldChannelName(const char * key, const char * name);
        void MonitorStatus(void);
        void bsssCallback(void *p, unsigned size);
        void bldCallback(void *p, unsigned size);
        serviceType_t getServiceType();
    private:

        char * pvaBaseName;
        uint64_t channelSevr;        
        void* pVoidsocketAPI[NUM_EDEF_MAX];
        uint32_t *bldPacketPayload;       
        uint32_t channelMask;
        
        pvxs::server::SharedPV                pv;
        pvxs::server::Server server;

        ELLLIST *pChannelEllList;
        AcqService::AcqServiceYaml *pService[MAX_MOD_NUM];

        void SetupAsynParams(serviceType_t type);
        void SetRate(int chn);
        void SetDest(int chn);
        void SetChannelSevr(int chn, int sevr);
        int  GetChannelSevr(int chn);

        void printMap();
        void llrfPerformChecks (const channelList_t*, const channelList_t*, int);
        void llrfCalcPhaseAmp  (signed short, signed short, double&, double&);

        serviceType_t serviceType;
        int           numMod;

        // LLRF specific functions
        float fastArcTan           (float x         );
        float atan2_approximation1 (float y, float x);
        float atan2_approximation2 (float y, float x);
        float atan2_approximation3 (float y, float x);
        float atan2_approximation4 (float y, float x);

    protected:
    
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstServiceParam;
        #define FIRST_SERVICE_PARAM    firstServiceParam
#endif /* asyn version check, under 4.32 */

        // Service status monitoring
        int p_currPacketSize[MAX_MOD_NUM];
        int p_currPacketStatus[MAX_MOD_NUM];
        int p_currPulseIdL[MAX_MOD_NUM];
        int p_currTimeStampL[MAX_MOD_NUM];
        int p_currDelta[MAX_MOD_NUM];
        int p_packetCount[MAX_MOD_NUM];
        int p_paused[MAX_MOD_NUM];
        int p_diagnClockRate[MAX_MOD_NUM];
        int p_diagnStrobeRate[MAX_MOD_NUM];
        int p_eventSel0Rate[MAX_MOD_NUM];

        // Service Status Control
        int p_packetSize;
        int p_enable;

        // Rate Limit BSSS specific
        int p_rateLimitBsss;

        // Service Rate Controls
        int p_edefEnable[NUM_EDEF_MAX];
        int p_rateMode[NUM_EDEF_MAX];
        int p_fixedRate[NUM_EDEF_MAX];
        int p_acRate[NUM_EDEF_MAX];
        int p_tSlotMask[NUM_EDEF_MAX];
        int p_expSeqNum[NUM_EDEF_MAX];
        int p_expSeqBit[NUM_EDEF_MAX];
        int p_destMode[NUM_EDEF_MAX];
        int p_destMask[NUM_EDEF_MAX];
        int p_rateLimit[NUM_EDEF_MAX];

        // bld specific
        int p_multicastAddr[NUM_EDEF_MAX];
        int p_multicastPort[NUM_EDEF_MAX];



#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int lastServiceParam;
        #define LAST_SERVICE_PARAM     lastServiceParam
#endif /* asyn version check, under 4.32 */
};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_SERVICE_DET_PARAMS ((int) (&LAST_SERVICE_PARAM - &FIRST_SERVICE_PARAM -1))
#endif /* asyn version check, under 4.32 */


#define CURRPACKETSIZE_STR      "%s_currPacketSize"
#define CURRPACKETSTATUS_STR    "%s_currPacketStatus"
#define CURRPULSEIDL_STR        "%s_currPulseIdL"
#define CURRTIMESTAMPL_STR      "%s_currTimeStampL"
#define CURRDELTA_STR           "%s_currDelta"
#define PACKETCOUNT_STR         "%s_packetCount"
#define PAUSED_STR              "%s_paused"
#define DIAGNCLOCKRATE_STR      "%s_diagnClockRate"
#define DIAGNSTROBERATE_STR     "%s_diagnStrobeRate"
#define EVENTSEL0RATE_STR       "%s_eventSel0Rate"

#define PACKETSIZE_STR          "%s_packetSize"
#define ENABLE_STR              "%s_enable"
#define CHANNELMASK_STR         "%s_channelMask_%s"
#define CHANNELSEVR_STR         "%s_channelSevr_%s"


#define EDEFENABLE_STR        "%s_edefEnable_%d"
#define RATEMODE_STR          "%s_rateMode_%d"
#define FIXEDRATE_STR         "%s_fixedRate_%d"
#define ACRATE_STR            "%s_acRate_%d"
#define TSLOTMASK_STR         "%s_tSlotMask_%d"
#define EXPSEQNUM_STR         "%s_expSeqNum_%d"
#define EXPSEQBIT_STR         "%s_expSeqBit_%d"
#define DESTMODE_STR          "%s_destMode_%d"
#define DESTMASK_STR          "%s_destMask_%d"
#define RATELIMIT_STR         "%s_rateLimit_%d"
#define RATELIMIT_PM_STR      "%s_rateLimit"        // use for BSSS rate limit, share a single PV for two modules

#define BSSSPV_STR            "%s_bsss_%d"
#define BSSSPID_STR           "%s_bsssPID_%d"

#define BLDMULTICASTADDR_STR  "bldMulticastAddress_%d"
#define BLDMULTICASTPORT_STR  "bldMulticastPort_%d"

#define BLD_STR "BLD"
#define BSSS_STR "BSSS"        // use for BSSS PVs
#define BSSS0_STR "BSSS0"      // use for BSSS packet control/status,  per module 
#define BSSS1_STR "BSSS1"      // use for BSSS packet control/status,  per moudle  


#endif /* BSSS_ASYN_DRIVER_H */


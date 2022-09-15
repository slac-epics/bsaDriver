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

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <ellLib.h>

#include <AcqServiceYaml.hh>
#include <BsssYaml.hh>
#include <BldYaml.hh>

#define NUM_BSSS_DATA_MAX    31
#define NUM_EDEF_MAX         16

#define IDX_NSEC 0
#define IDX_SEC  1
#define IDX_PIDL 2
#define IDX_PIDU 3
#define IDX_CHN_MASK 4
#define IDX_SVC_MASK 5
#define IDX_DATA 6
#define IDX_SEVR_MASK(size)  (size/4 -2)

typedef enum {
    int32_service,
    uint32_service,
    uint64_service,
    float32_service,
    fault_service
} serviceDataType_t;

typedef enum {
    bld,
    bsss
} serviceType_t;

typedef struct {
    ELLNODE node;
    char    service_name[64];

    int     index;

    int     p_firstParam;
    int     p_channelMask;
    int     p_channelSevr;
    int     p_service[NUM_EDEF_MAX];
    int     p_servicePID[NUM_EDEF_MAX];
    int     p_lastParam;
    
    serviceDataType_t type;
    double  *pslope;
    double  *poffset;
    
    char    pname_service[NUM_EDEF_MAX][64];
    char    pname_servicePID[NUM_EDEF_MAX][64];
} serviceList_t;


class serviceAsynDriver: asynPortDriver {

    public:
        serviceAsynDriver(const char *protName, const char *reg_path, const int num_dyn_param, ELLLIST *pServiceList, serviceType_t type, const char *named_root = NULL);
        ~serviceAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        void MonitorStatus(void);
        void bsssCallback(void *p, unsigned size);

    private:

        uint64_t channelSevr;        

        ELLLIST *pServiceEllList;
        AcqService::AcqServiceYaml *pService;

        void SetupAsynParams(serviceType_t type);
        void SetRate(int chn);
        void SetDest(int chn);
        void SetChannelSevr(int chn, int sevr);
        int  GetChannelSevr(int chn);


    protected:
    serviceType_t serviceType;
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstServiceParam;
        #define FIRST_SERVICE_PARAM    firstServiceParam
#endif /* asyn version check, under 4.32 */

        // Service status monitoring
        int p_currPacketSize;
        int p_currPacketStatus;
        int p_currPulseIdL;
        int p_currTimeStampL;
        int p_currDelta;
        int p_packetCount;
        int p_paused;
        int p_diagnClockRate;
        int p_diagnStrobeRate;
        int p_eventSel0Rate;

        // Service Status Control
        int p_packetSize;
        int p_enable;


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

#define BSSSPV_STR            "%s_bsss_%d"
#define BSSSPID_STR           "%s_bsssPID_%d"

#define BLD_STR "BLD_"
#define BSSS_STR "BSSS_"


#endif /* BSSS_ASYN_DRIVER_H */


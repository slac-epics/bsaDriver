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

#include <BsssYaml.hh>

#define NUM_BSSS_DATA_MAX    31

typedef enum {
    int32_bsss,
    uint32_bsss,
    uint64_bsss,
    float32_bsss,
    fault_bsss
} bsssDataType_t;

typedef struct {
    ELLNODE node;
    char    bsss_name[64];

    int     p_firstParam;
    int     p_bsss[NUM_BSSS_CHN];
    int     p_bsssPID[NUM_BSSS_CHN];
    int     p_lastParam;
    
    bsssDataType_t type;
    double  *pslope;
    double  *poffset;

    char    pname_bsss[NUM_BSSS_CHN][64];
    char    pname_bsssPID[NUM_BSSS_CHN][64];
} bsssList_t;


class bsssAsynDriver: asynPortDriver {

    public:
        bsssAsynDriver(const char *protName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsssList, const char *named_root = NULL);
        ~bsssAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    private:
        ELLLIST *pBsssEllList;
        Bsss::BsssYaml *pBsss;
        

        void SetupAsynParams(void);
        void SetRate(int chn);
        void SetDest(int chn);
        void MonitorStatus(void);

    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstBsssParam;
        #define FIRST_BSSS_PARAM    firstBsssParam
#endif /* asyn version check, under 4.32 */

        // BSSS status monitoring
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

        // BSSS Status Control
        int p_packetSize;
        int p_enable;
        int p_channelMask[NUM_BSSS_DATA_MAX];
        int p_channelSevr[NUM_BSSS_DATA_MAX];

        // BSSS Rate Controls
        int p_edefEnable;
        int p_rateMode[NUM_BSSS_CHN];
        int p_fixedRate[NUM_BSSS_CHN];
        int p_acRate[NUM_BSSS_CHN];
        int p_tSlotMask[NUM_BSSS_CHN];
        int p_expSeqNum[NUM_BSSS_CHN];
        int p_expSeqBit[NUM_BSSS_CHN];
        int p_destMode[NUM_BSSS_CHN];
        int p_destMask[NUM_BSSS_CHN];


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int lastBsssParam;
        #define LAST_BSSS_PARAM     lastBsssParam
#endif /* asyn version check, under 4.32 */
};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_BSSS_DET_PARAMS ((int) (&LAST_BSSS_PARAM - &FIRST_BSSS_PARAM -1))
#endif /* asyn version check, under 4.32 */


#define CURRPACKETSIZE_STR      "currPacketSize"
#define CURRPACKETSTATUS_STR    "currPacketStatus"
#define CURRPULSEIDL_STR        "currPulseIdL"
#define CURRTIMESTAMPL_STR      "currTimeStampL"
#define CURRDELTA_STR           "currDelta"
#define PACKETCOUNT_STR         "packetCount"
#define PAUSED_STR              "paused"
#define DIAGNCLOCKRATE_STR      "diagnClockRate"
#define DIAGNSTROBERATE_STR     "diagnStrobeRate"
#define EVENTSEL0RATE_STR       "eventSel0Rate"

#define PACKETSIZE_STR          "packetSize"
#define ENABLE_STR              "enable"
#define CHANNELMASK_STR         "channelMask_%d"
#define CHANNELSEVR_STR         "channelSevr_%d"


#define EDEFENABLE_STR        "edefEnable"
#define RATEMODE_STR          "rateMode_%d"
#define FIXEDRATE_STR         "fixedRate_%d"
#define ACRATE_STR            "acRate_%d"
#define TSLOTMASK_STR         "tSlotMask_%d"
#define EXPSEQNUM_STR         "expSeqNum_%d"
#define EXPSEQBIT_STR         "expSeqBit_%d"
#define DESTMODE_STR          "destMode_%d"
#define DESTMASK_STR          "destMask_%d"

#define BSSSPV_STR            "%s_bsss_%d"
#define BSSSPID_STR           "%s_bsssPID_%d"


#endif /* BSSS_ASYN_DRIVER_H */


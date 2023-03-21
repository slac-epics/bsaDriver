#ifndef BSAS_ASYN_DRIVER_H
#define BSAS_ASYN_DRIVER_H


#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <new>
#include <arpa/inet.h>
#include <sstream>
#include <vector>
#include <map>

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <ellLib.h>


#include <BsasYaml.hh>

#define PVNAME_LEN              128

#define HW_CHANNELS             31
#define NUM_BSAS_MODULES        4
#define NUM_BSAS_CTRL           3    // acquire, rowAdvance, tableReset

#define MAXROWS                  1000   // max row in NTTable

#define NTTBL_ID                "epics:nt/NTTable:1.0"
#define LABEL_FIELD             "labels"
#define VALUE_FIELD             "value"
#define SEC_COL                 "secondsPastEpoch"
#define NSEC_COL                "nanoseconds"
#define PID_COL                 "pulseId"
#define CNT_STR                 "pv%d_cnt"
#define VAL_STR                 "pv%d_val"
#define AVG_STR                 "pv%d_avg"
#define RMS_STR                 "pv%d_rms"
#define MIN_STR                 "pv%d_min"
#define MAX_STR                 "pv%d_max"

#define ENABLE  1
#define DISABLE 0

#define BLOCK_WIDTH_2   2
#define BLOCK_WIDTH_16  16
#define BLOCK_WIDTH_32  32
#define BLOCK_WIDTH_64  64 

#define KEEP_LSB_2    0x00000003 
#define KEEP_LSB_16   0x0000ffff 
#define DEFAULT_MASK  0xffffffff

#define M_PI_DEGREES 180.0

typedef enum {
    uint2_bsas,
    uint16_bsas,
    int32_bsas,
    uint32_bsas,
    uint64_bsas,
    float32_bsas,
    fault_bsas,
    llrfAmp_bsas,
    llrfPhase_bsas
} bsasDataType_t;

static std::map<bsasDataType_t, int> bsasBitMap = {{uint2_bsas,   BLOCK_WIDTH_2 },
                                                   {uint16_bsas,  BLOCK_WIDTH_16},
                                                   {int32_bsas,   BLOCK_WIDTH_32},
                                                   {uint32_bsas,  BLOCK_WIDTH_32},
                                                   {uint64_bsas,  BLOCK_WIDTH_64},
                                                   {float32_bsas, BLOCK_WIDTH_32}};

static std::map<int, std::vector<int>> bsasHwChannelUsage;

typedef enum {
    acquire_idx, 
    rowAdvance_idx,
    tableReset_idx
} ctrlIdx_t;

typedef struct {
    ELLNODE               node;
    char                  bsas_name[PVNAME_LEN];
    char                  pv_name[PVNAME_LEN];

    int                   swChIndex;
    int                   hwChIndex;
    int                   p_firstParam;
    int                   p_channelMask;
    int                   p_channelSevr;
    int                   p_ts[NUM_BSAS_MODULES];
    int                   p_pid[NUM_BSAS_MODULES];
    int                   p_cnt[NUM_BSAS_MODULES];
    int                   p_avg[NUM_BSAS_MODULES];
    int                   p_rms[NUM_BSAS_MODULES];
    int                   p_min[NUM_BSAS_MODULES];
    int                   p_max[NUM_BSAS_MODULES];
    int                   p_val[NUM_BSAS_MODULES];
    int                   p_lastParam;

    bsasDataType_t        type;
    double                *pslope;
    double                *poffset;

    char                  pname_ts[PVNAME_LEN];
    char                  pname_pid[PVNAME_LEN];
    char                  pname_cnt[PVNAME_LEN];
    char                  pname_avg[PVNAME_LEN];
    char                  pname_rms[PVNAME_LEN];
    char                  pname_min[PVNAME_LEN];
    char                  pname_max[PVNAME_LEN];
    char                  pname_val[PVNAME_LEN];
    
    char                  fname_cnt[PVNAME_LEN];
    char                  fname_avg[PVNAME_LEN];
    char                  fname_rms[PVNAME_LEN];
    char                  fname_min[PVNAME_LEN];
    char                  fname_max[PVNAME_LEN];
    char                  fname_val[PVNAME_LEN];
} bsasList_t;


typedef struct {
    int                   p_edefEnable;
    int                   p_rateMode;
    int                   p_fixedRate;
    int                   p_acRate;
    int                   p_tSlotMask;
    int                   p_expSeqNum;
    int                   p_expSeqBit;
    int                   p_destMode;
    int                   p_destMask;
} control_asynParam_t;

typedef struct {
    control_asynParam_t   p_acquire;
    control_asynParam_t   p_rowAdvance;
    control_asynParam_t   p_tableReset;
} ctrlName_asynParam_t;


typedef struct {
    control_asynParam_t   p_idx[NUM_BSAS_CTRL];
} ctrlIdx_asynParam_t;

typedef union {
    ctrlName_asynParam_t  name;
    ctrlIdx_asynParam_t   idx;
} module_asynParam_t;


typedef struct __attribute__ ((packed)) {
    uint64_t timestamp:    64;
    uint64_t pulse_id:     64;
    uint32_t channelMask:  32;
    uint16_t row_number:   16;
    uint8_t  table_count:   4;
    uint8_t  edef_index:    4;
    uint8_t  byte_pad:      8;
} header_t;     /*  24 bytes header */

typedef struct __attribute__ ((packed)) {
    uint16_t sample_count: 13;
    bool     exception_sum: 1;
    bool     exception_var: 1;
    bool     flag_fixed:    1;
    uint32_t val:          32;
    uint32_t sum:          32;
    uint64_t sum_square:   48;
    uint32_t min:          32;
    uint32_t max:          32;       
} payload_t;     /* 24 bytes payload for each channel */

typedef struct __attribute__((packed)) {
    header_t      hd;      // bsas header
    payload_t     pl[];    // bsas payloader (arbitrary length
} packet_t;


class chnCol {
    int      last_row;
    uint32_t cnt[MAXROWS];
    double   val[MAXROWS];
    double   avg[MAXROWS];
    double   rms[MAXROWS];
    double   min[MAXROWS];
    double   max[MAXROWS];

    public:
    void     init(void);
    int      store(int row, uint32_t cnt, double val, double avg, double rms, double min, double max);
    void     pushPV(pvxs::Value *ppv, void *pChn);
    
};

class ntTbl {
    int      last_row;
//    uint64_t timestamp[MAXROWS];
    uint32_t sec[MAXROWS];
    uint32_t nsec[MAXROWS];
    uint64_t pulse_id[MAXROWS];
    chnCol   *col;
    int      num_chn;
    public:
    ntTbl(int num_chn);
    ~ntTbl() {}

    void init(void);
    int store(int row, uint64_t timestamp, uint64_t pulse_id);
    int store(int col, int row, uint32_t cnt, double val, double avg, double rms, double min, double max);
    void pushPV(pvxs::Value *ppv, std::vector<void *> *pActiveChannels);
};


class edefNTTbl {
    int   table_count;
    int   swing_idx;
    ntTbl *pTbl[2];

    // NTTable PV stuff
    pvxs::shared_array<const std::string> _labels;  
    pvxs::TypeDef                         _def;
    pvxs::Value                           _initial;
    pvxs::server::SharedPV                pv;

    public:
    edefNTTbl(int num_chn);
    ~edefNTTbl() {}

    pvxs::server::SharedPV lateInit(const char *ntTablename, std::vector<void *> *pActiveChannels);
    void pushPV(std::vector<void *> *pActiveChannels);
    inline void swing(void);
    int checkUpdate(int table_count);
    int store(int row, uint64_t timestamp, uint64_t pulse_id);
    int store(int col, int row, uint32_t cnt, double val, double avg, double rms, double min, double max);
};



class bsasAsynDriver: asynPortDriver {
    public:
        bsasAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsasList, 
                       const char *ntTable_name1,
                       const char *ntTable_name2,
                       const char *ntTable_name3,
                       const char *ntTable_name4,
                       const char *named_root = NULL);
        ~bsasAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        void bsasCallback(void *p, unsigned size);

        void lateInit(void);
    private:
        ELLLIST                 *pBsasEllList;
        Bsas::BsasModuleYaml    *pBsas[NUM_BSAS_MODULES];

        char              ntTableName[NUM_BSAS_MODULES][PVNAME_LEN];
        uint32_t          channelMask;
        uint64_t          channelSevr;
        std::vector <void *> *activeChannels;
        edefNTTbl         *pEdefNTTbl[NUM_BSAS_MODULES];

        void              SetupAsynParams(void);
        void              SetRate(int module, ctrlIdx_t ctrl);
        void              SetDest(int module, ctrlIdx_t ctrl);
        void              EdefEnable(int module, ctrlIdx_t ctrl, int enable);
        void              SetChannelMask(int chn, bool flag);
        void              SetChannelMask(uint32_t mask);
        void              SetChannelSevr(int chn, uint64_t sevr);
        void              SetChannelSevr(uint64_t sevr);

        void printMap();
        void llrfPerformChecks (const bsasList_t*, const bsasList_t*, int);
        void llrfCalcPhaseAmp  (signed short, signed short, double&, double&);

    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstBsasParam;
        #define FIRST_BSAS_PARAM    firstBsasParam
#endif /* asyn version check, under 4.32 */

        module_asynParam_t p_module[NUM_BSAS_MODULES];

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int lastBsasParam;
        #define LAST_BSAS_PARAM     lastBsasParam
#endif /* asyn version check, under 4.32 */

};



#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_BSAS_DET_PARAMS ((int) (&LAST_BSAS_PARAM - &FIRST_BSAS_PARAM -1))
#endif /* asyn version check, under 4.32 */

#define PARAM_ACQUIRE_STR     "acq"
#define PARAM_ROWADV_STR      "rowAdv"
#define PARAM_TBLRESET_STR    "tblReset"

#define CHANNEL_MASK_STR      "channelMask_%s"
#define CHANNEL_SEVR_STR      "channelSevr_%s"

#define EDEFENABLE_STR        "edefEnable_%d_%s"
#define RATEMODE_STR          "rateMode_%d_%s"
#define FIXEDRATE_STR         "fixedRate_%d_%s"
#define ACRATE_STR            "acRate_%d_%s"
#define TSLOTMASK_STR         "tSlotMask_%d_%s"
#define EXPSEQNUM_STR         "expSeqNum_%d_%s"
#define EXPSEQBIT_STR         "expSeqBit_%d_%s"
#define DESTMODE_STR          "destMode_%d_%s"
#define DESTMASK_STR          "destMask_%d_%s"

#endif /* BSAS_ASYN_DRIVER_H */

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

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <ellLib.h>


#include <BsasYaml.hh>


#define NUM_BSAS_DATA_MAX       31
#define NUM_BSAS_MODULES        4
#define NUM_BSAS_CTRL           3    // acquire, rowAdvance, tableReset



typedef enum {
    int32_bsas,
    uint32_bsas,
    uint64_bsas,
    float32_bsas,
    fault_bsas
} bsasDataType_t;

typedef enum {
    acquire_idx, 
    rowAdvance_idx,
    tableReset_idx
} ctrlIdx_t;


typedef struct {
    ELLNODE               node;
    char                  bsas_name[64];
    char                  pv_name[128];

    int                   p_firstParam;
    int                   p_ts[NUM_BSAS_MODULES];
    int                   p_pid[NUM_BSAS_MODULES];
    int                   p_cnt[NUM_BSAS_MODULES];
    int                   p_avg[NUM_BSAS_MODULES];
    int                   p_rms[NUM_BSAS_MODULES];
    int                   p_min[NUM_BSAS_MODULES];
    int                   p_max[NUM_BSAS_MODULES];
    int                   p_lastParam;

    bsasDataType_t        type;
    double                *pslope;
    double                *poffset;

    char                  pname_ts[NUM_BSAS_MODULES][64];
    char                  pname_pid[NUM_BSAS_MODULES][64];
    char                  pname_cnt[NUM_BSAS_MODULES][64];
    char                  pname_avg[NUM_BSAS_MODULES][64];
    char                  pname_rms[NUM_BSAS_MODULES][64];
    char                  pname_min[NUM_BSAS_MODULES][64];
    char                  pname_max[NUM_BSAS_MODULES][64];
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


class bsasAsynDriver: asynPortDriver {
    public:
        bsasAsynDriver(const char *portName, const char *reg_path, const int num_dyn_param, ELLLIST *pBsasList, const char *named_root = NULL);
        ~bsasAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        void bsasCallback(void *p, unsigned size);
    private:
        ELLLIST                 *pBsasEllList;
        Bsas::BsasModuleYaml    *pBsas[NUM_BSAS_MODULES];

        void              SetupAsynParams(void);
        void              SetRate(int module, ctrlIdx_t ctrl);
        void              SetDest(int module, ctrlIdx_t ctrl);
        void              EdefEnable(int module, ctrlIdx_t ctrl, int enable);

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
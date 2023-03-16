#ifndef BSA_ASYN_DRIVER_H
#define BSA_ASYN_DRIVER_H


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

#include <BsaField.hh>
#include <Processor.hh>

#include "devScBsa.h"


#define  START_BSA_ARRAY   21             // making index for database temlate, starting 21
#define  MAX_BSA_ARRAY     48             // internal index, start from 0
#define  MAX_BSA_LENGTH    20000

#define  FLTB_ARRAY0       44             // internal index, database template should have index n + 21 = 65
#define  FLTB_ARRAY1       45             // internal index, database template should have index n + 21 = 66
#define  FLTB_ARRAY2       46             // internal index, database template should have index n + 21 = 67
#define  FLTB_ARRAY3       47             // internal index, database template should have index n + 21 = 68
#define  MAX_FLTB_LENGTH   1000000

#define  UINT2STRING     "uint2"
#define  INT16STRING     "int16"
#define  UINT16STRING    "uint16"
#define  INT32STRING     "int32"
#define  UINT32STRING    "uint32"
#define  UINT64STRING    "uint64"
#define  FLOAT32STRING   "float32"

#define HW_CHANNELS     31

#define BLOCK_WIDTH_2   2
#define BLOCK_WIDTH_16  16
#define BLOCK_WIDTH_32  32

#define KEEP_LSB_2    0x00000003 
#define KEEP_LSB_16   0x0000ffff 
#define DEFAULT_MASK  0xffffffff

extern "C" {
// interface for BSSS driver
ELLLIST * find_bsaChannelList(const char *port_name);
}

struct ChannelDataStruct {
    unsigned n;
    double mean;
    double rms2;
    ChannelDataStruct(unsigned nVal, double meanVal, double rms2Val)
    : n    (nVal   ),
      mean (meanVal),
      rms2 (rms2Val)
      {
      }
};

class bsaAsynDriver;


class BsaField : public Bsa::Field {
    public:
        BsaField(char *name, int index, int p_num, int p_mean, int p_rms2, double * p_slope, double * p_offset, Bsa::bsaDataType_t * p_type,
                  devBsaPvt_t *ppvt_num, devBsaPvt_t *ppvt_mean, devBsaPvt_t *ppvt_rms2);
        const char *name() const { return _name.c_str(); }
        const int get_p_num()  const { return _p_num; }
        const int get_p_mean() const { return _p_mean; }
        const int get_p_rms2() const { return _p_rms2; }

        double * get_p_slope() const { return _p_slope; }
        double * get_p_offset() const { return _p_offset; }
        unsigned get_max_size(void) { return max_size; }

        Bsa::bsaDataType_t * get_p_type() const { return _p_type; }

        devBsaPvt_t *get_ppvt_num() const  { return _ppvt_num; }
        devBsaPvt_t *get_ppvt_mean() const { return _ppvt_mean; }
        devBsaPvt_t *get_ppvt_rms2() const { return _ppvt_rms2; }

        std::vector <BsaField *> slaveField;

    private:
        std::string _name;

        int _p_num;    // asyn parameter index from bsa driver
        int _p_mean;   // asyn parameter index from bsa driver
        int _p_rms2;   // asyn parameter index from bsa driver

        double *_p_slope;    // slope data pointer from bsa driver
        double *_p_offset;   // offset data pointer from bsa driver

        Bsa::bsaDataType_t *_p_type;

        devBsaPvt_t *_ppvt_num;
        devBsaPvt_t *_ppvt_mean;
        devBsaPvt_t *_ppvt_rms2;

        unsigned max_size;
};


class BsaPv : public Bsa::Pv {
    public:
        BsaPv(Bsa::Field &f, bsaAsynDriver *pBsaDrv);
        const Bsa::Field& field() const { return _f; }
        const char *name();
        void clear();
        void setTimestamp(unsigned sec, unsigned nsec);
        void append();
        void append(unsigned n, double mean, double rms2);
        void flush();

        devBsaPvt_t *get_ppvt_num() { return _ppvt_num; }
        devBsaPvt_t *get_ppvt_mean() { return _ppvt_mean; }
        devBsaPvt_t *get_ppvt_rms2() { return _ppvt_rms2; }

        std::vector <BsaPv *> slavePv;

    private:
        bsaAsynDriver *pBsaDrv;
        Bsa::Field& _f;
        unsigned _ts_sec;
        unsigned _ts_nsec;
        std::vector <unsigned> _n;
        std::vector <double> _mean;
        std::vector <double> _rms2;

        unsigned size, loc, max_size;

        // asyn parameters from bsa driver
        int _p_num;
        int _p_mean;
        int _p_rms2;

        // slope and offset data pointer from bsa driver
        double * _p_slope;
        double * _p_offset;

        devBsaPvt_t *_ppvt_num;
        devBsaPvt_t *_ppvt_mean;
        devBsaPvt_t *_ppvt_rms2;
};


class BsaPvArray : public Bsa::PvArray {
    public:
        BsaPvArray(unsigned array, const std::vector <Bsa::Pv*>& pvs, int p_pid_UL, bsaAsynDriver *pBsaDrv, devBsaPvt_t *ppvt);
        unsigned array() const { return _array; }
        void reset(unsigned sec, unsigned nsec);
        void set(unsigned sec, unsigned nsec);
        void append(uint64_t pulseId);
        std::vector <Bsa::Pv*> pvs();
        void flush();

        void procChannelData(unsigned n, double mean, double rms2, bool done);

        unsigned get_ts_sec(void)  { return _ts_sec; }
        unsigned get_ts_nsec(void) { return _ts_nsec; }

        unsigned get_max_size(void) { return max_size; }

    private:
        bsaAsynDriver *pBsaDrv;
        unsigned    _array;
        unsigned    _ts_sec;
        unsigned    _ts_nsec;
        devBsaPvt_t *_ppvt_pid;
        std::vector <uint64_t> _pid;

        ChannelDataStruct* _rawChannelData[HW_CHANNELS]; 

        unsigned size, loc, max_size;

        const std::vector <Bsa::Pv*>& _pvs;

        int _p_pid_UL;
};








typedef struct {
    ELLNODE  node;
    ELLLIST  *pSlaveEllList;  // slave node
    char     bsa_name[64];    // bsa name
    char     bsa_type[32];    // bsa datatype

    int      firstParam;
    int      p_num[MAX_BSA_ARRAY];           // asyn parameter for number of average, asynFloat64Array, RO
    int      p_mean[MAX_BSA_ARRAY];          // asyn parameter for average value,     asynFloat64Array, RO
    int      p_rms2[MAX_BSA_ARRAY];          // asyn parameter for rms2 value,        asynFloat64Array, RO
    int      p_slope;                        // asyn parameter for linear conversion, asynFloat64, RW
    int      p_offset;                       // asyn parameter for linear conversion, asynFloat64, RW
    int      lastParam;

    double   slope;
    double   offset;

    Bsa::bsaDataType_t type;

    devBsaPvt_t ppvt_num[MAX_BSA_ARRAY];
    devBsaPvt_t ppvt_mean[MAX_BSA_ARRAY];
    devBsaPvt_t ppvt_rms2[MAX_BSA_ARRAY];

    char     pname_num[MAX_BSA_ARRAY][64];
    char     pname_mean[MAX_BSA_ARRAY][64];
    char     pname_rms2[MAX_BSA_ARRAY][64];

    char     pname_slope[64];
    char     pname_offset[64];
    bool     doNotTouch;
} bsaList_t;




class bsaAsynDriver:asynPortDriver {

public:

#ifndef HAVE_YAML
    bsaAsynDriver(const char *portName, const char *ipString, const int num_dynamic_params);
#else
    bsaAsynDriver(const char *portName, const char *path_reg, const char *path_ram, const int num_dynamic_params, ELLLIST *pBsaList, const char *named_root = NULL);
#endif  /* HAVE_YAML */
	  ~bsaAsynDriver();
    void SetupAsynParams(void);
    void SetupFields(void);
    void SetupPvs(void);
    void SetupPvArray(void);

    int  BsaRetreivePoll(void);
    int  bsaAsynDriverEnable(void);
    int  bsaAsynDriverDisable(void);

    devBsaPvt_t *findPvt(char *key, char *param, int edef);


    asynStatus flush(devBsaPvt_t *ppvt);

    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
   //	asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

private:
    ELLLIST *pBsaEllList;
    Bsa::Processor* pProcessor;
    std::vector <Bsa::Field*> fields[MAX_BSA_ARRAY];
    std::vector <Bsa::Pv*> pvs[MAX_BSA_ARRAY];
    std::vector <BsaPvArray*> pBsaPvArray;
    devBsaPvt_t ppvt_pid[MAX_BSA_ARRAY];



protected:
    int bsa_enable;
//
// parameter section for asynPortDriver,
// just static parameter should be listed here
//
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
    int firstBsaParam;
    #define FIRST_BSA_PARAM    firstBsaParam
#endif /* asyn version check, under 4.32 */
    int p_pid_UL[MAX_BSA_ARRAY];      // asynInt32Array, RO
    int p_enable;
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
    int lastBsaParam;
    #define LAST_BSA_PARAM     lastBsaParam
#endif /* asyn version check, under 4.32 */
};

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_BSA_DET_PARAMS ((int) (&LAST_BSA_PARAM - &FIRST_BSA_PARAM -1))
#endif /* asyn version check, under 4.32 */

#define pidString     "BSAPID_%d"
#define numString     "%s_BSANUM_%d"
#define meanString    "%s_BSAMEAN_%d"
#define rms2String    "%s_BSARMS2_%d"

#define slopeString    "%s_slope"
#define offsetString   "%s_offset"

#define enableString   "bsa_enable"

#endif  /* BSA_ASYN_DRIVER_H */

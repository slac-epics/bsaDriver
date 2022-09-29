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


#define  START_BSA_ARRAY   21
#define  MAX_BSA_ARRAY     64
#define  MAX_BSA_LENGTH    20000

#define  FLTB_ARRAY0       60
#define  FLTB_ARRAY1       61
#define  FLTB_ARRAY2       62
#define  FLTB_ARRAY3       63
#define  MAX_FLTB_LENGTH   1000000

#define  INT32STRING     "int32"
#define  UINT32STRING    "uint32"
#define  UINT64STRING    "uint64"
#define  FLOAT32STRING   "float32"

extern "C" {
// interface for BSSS driver
ELLLIST * find_bsaChannelList(const char *port_name);
}

typedef enum {
    int32,
    uint32,
    uint64,
    float32,
    fault
} bsaDataType_t;


class bsaAsynDriver;


class BsaField : public Bsa::Field {
    public:
        BsaField(char *name, int index, int p_num, int p_mean, int p_rms2, double * p_slope, double * p_offset, bsaDataType_t * p_type);
        const char *name() const { return _name.c_str(); }
        const int get_p_num()  const { return _p_num; }
        const int get_p_mean() const { return _p_mean; }
        const int get_p_rms2() const { return _p_rms2; }

        double * get_p_slope() const { return _p_slope; }
        double * get_p_offset() const { return _p_offset; }
        unsigned get_max_size(void) { return max_size; }

        bsaDataType_t * get_p_type() const { return _p_type; }

        std::vector <BsaField *> slaveField;

    private:
        std::string _name;

        int _p_num;    // asyn parameter index from bsa driver
        int _p_mean;   // asyn parameter index from bsa driver
        int _p_rms2;   // asyn parameter index from bsa driver

        double *_p_slope;    // slope data pointer from bsa driver
        double *_p_offset;   // offset data pointer from bsa driver

        bsaDataType_t *_p_type;

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

        bsaDataType_t *_p_type;


};


class BsaPvArray : public Bsa::PvArray {
    public:
        BsaPvArray(unsigned array, const std::vector <Bsa::Pv*>& pvs, int p_pid_UL, bsaAsynDriver *pBsaDrv);
        unsigned array() const { return _array; }
        void reset(unsigned sec, unsigned nsec);
        void set(unsigned sec, unsigned nsec);
        void append(uint64_t pulseId);
        std::vector <Bsa::Pv*> pvs();
        void flush();

        unsigned get_ts_sec(void)  { return _ts_sec; }
        unsigned get_ts_nsec(void) { return _ts_nsec; }

        unsigned get_max_size(void) { return max_size; }

    private:
        bsaAsynDriver *pBsaDrv;
        unsigned    _array;
        unsigned    _ts_sec;
        unsigned    _ts_nsec;
        std::vector <uint64_t> _pid;

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

    bsaDataType_t type;

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

    asynStatus flush(double *pData, unsigned size, int param);
    asynStatus flush(unsigned *pData, unsigned size, int param);
    asynStatus flush(int *pData, unsigned size, int param);
    asynStatus flush(uint64_t *pData, unsigned size, int param);

    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
   //	asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

private:
    ELLLIST *pBsaEllList;
    Bsa::Processor* pProcessor;
    std::vector <Bsa::Field*> fields[MAX_BSA_ARRAY];
    std::vector <Bsa::Pv*> pvs[MAX_BSA_ARRAY];
    std::vector <BsaPvArray*> pBsaPvArray;



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

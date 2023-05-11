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

#include <BsaField.hh>
#include <Processor.hh>

#include <bsaAsynDriver.h>

#include <yamlLoader.h>

#include "devScBsa.h"


#define   NORM_INTV   (.1)
#define   FAULT_INTV  (5.)
#define   NORMAL      0
#define   FAULT       1

static const  char *driverName = "bsaAsynDriver";
static char port_name[32];
static char ip_string[32];
static char reg_path_string[256];
static char ram_path_string[256];

static double poll_intv = NORM_INTV;

// static class bsaAsynDriver *pBsaDrv = NULL;  /* will be removed */

//static ELLLIST *pBsaEllList = NULL;
static void *_pBsaBridge = NULL;    /* will be removed */

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE       node;
    char          *named_root;
    char          *port;
    bsaAsynDriver *pBsaDrv;
    ELLLIST       *pBsaEllList;
    void          *_pBsaBridge;
    
} pDrvList_t;


static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsaDriver(init_drvList)");
        ellInit(pDrvEllList);
    }

    return;
}


static int prep_drvAnonimous(void)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsaDriver(prep_drvAnonimous)");

    p->named_root  = NULL;
    p->port        = NULL;
    p->pBsaDrv     = NULL;
    p->pBsaEllList = NULL;
    p->_pBsaBridge = NULL;

    ellAdd(pDrvEllList, &p->node);

    return ellCount(pDrvEllList);
}

static int prep_drvByPort(const char *port)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsaDriver(prep_drvByPort)");

    p->named_root  = NULL;
    p->port        = epicsStrDup(port);
    p->pBsaDrv     = NULL;
    p->pBsaEllList = NULL;
    p->_pBsaBridge = NULL;

    ellAdd(pDrvEllList, &p->node);

    return ellCount(pDrvEllList);

    
}


static int prep_drvByNamedRoot(const char *named_root)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bsaDriver(prep_drvByNamedRoot)");

    p->named_root = epicsStrDup(named_root);
    p->port        = NULL;
    p->pBsaDrv     = NULL;
    p->pBsaEllList = NULL;
    p->_pBsaBridge = NULL;

    ellAdd(pDrvEllList, &p->node);

    return ellCount(pDrvEllList);

}



static pDrvList_t *find_drvByPort(const char *port)
{
    init_drvList();
    pDrvList_t  *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->port && strlen(p->port) && !strcmp(p->port, port)) break; 
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}

static pDrvList_t *find_drvByNamedRoot(const char *named_root)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->named_root && strlen(p->named_root) && !strcmp(p->named_root, named_root)) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}


static pDrvList_t *find_drvLast(void)
{
    init_drvList();

    if(ellCount(pDrvEllList)) return (pDrvList_t *) ellLast(pDrvEllList);

    return (pDrvList_t *) NULL;
}



extern "C" { void *_getBsaBridge(void) { return _pBsaBridge; } }
extern "C" {
    static int bsa_length   = MAX_BSA_LENGTH;
    static int fltb_length  = MAX_FLTB_LENGTH;
    static int bsa_item_cnt   = 0;
    static int bsa_mem_alloc  = 0;

    epicsExportAddress(int, bsa_length);
    epicsExportAddress(int, fltb_length);
    epicsExportAddress(int, bsa_item_cnt);
    epicsExportAddress(int, bsa_mem_alloc);

}

static unsigned determine_max_size(unsigned array_index)
{
    return (array_index < FLTB_ARRAY0)? (unsigned) bsa_length: (unsigned) fltb_length;
}


BsaField::BsaField(char *name, int index, int p_num, int p_mean, int p_rms2, double * p_slope, double * p_offset, bsaDataType_t * p_type,
                   devBsaPvt_t *ppvt_num, devBsaPvt_t *ppvt_mean, devBsaPvt_t *ppvt_rms2)
{
    std::ostringstream o;
    o << name << index;
    _name = o.str();

    // copy asyn paramter indexes from bsa driver
    _p_num  = p_num;
    _p_mean = p_mean;
    _p_rms2 = p_rms2;

    // copy slope and offset data pointer from bsa driver
    _p_slope  = p_slope;
    _p_offset = p_offset;

    _p_type = p_type;

    _ppvt_num  = ppvt_num;
    _ppvt_mean = ppvt_mean;
    _ppvt_rms2 = ppvt_rms2;
  

    max_size = determine_max_size(index);
}


BsaPv::BsaPv (Bsa::Field& f, bsaAsynDriver *pBsaDrv) : _f(f), _n(0), _mean(0), _rms2(0), size(0), loc(0)
{
    this->pBsaDrv = pBsaDrv;
    BsaField *p = (BsaField *)&_f;

   // copy asyn paramter indexes from field calss
    _p_num  = p->get_p_num();
    _p_mean = p->get_p_mean();
    _p_rms2 = p->get_p_rms2();

    // copy slope and offfset data pointer from field class
    _p_slope  = p->get_p_slope();
    _p_offset = p->get_p_offset();

    _p_type = p->get_p_type();

    _ppvt_num  = p->get_ppvt_num();
    _ppvt_mean = p->get_ppvt_mean();
    _ppvt_rms2 = p->get_ppvt_rms2();

    max_size = p->get_max_size();


// reserve memory for better performance
// need to test to measure improvement
   _n.reserve(max_size *2);     _n.resize(max_size *2);      bsa_item_cnt++;
   _mean.reserve(max_size *2);  _mean.resize(max_size *2);   bsa_item_cnt++;
   _rms2.reserve(max_size *2);  _rms2.resize(max_size *2);   bsa_item_cnt++;

   for(unsigned i=0; i < p->slaveField.size(); i++) {
       slavePv.push_back(new BsaPv(*p->slaveField[i], this->pBsaDrv));
   }
}





const char* BsaPv::name()
{
    return _f.name();
}

void BsaPv::clear()
{
    size = 0;
    loc  = 0;

    for(unsigned i =0; i <slavePv.size(); i++) {
        slavePv[i]->clear();   /* execute clear() method for all of slaves */
    }
}


void BsaPv::setTimestamp(unsigned sec, unsigned nsec)
{
    _ts_sec  = sec;
    _ts_nsec = nsec;

    for(unsigned i =0; i< slavePv.size(); i++) {
        slavePv[i]->setTimestamp(sec, nsec);    /* execture setTimestamp() method for all of slaves */
    }
}

void BsaPv::append()
{

    _n[loc]    = _n[loc + max_size] = 0;
    _mean[loc] = _mean[loc + max_size] = nan("");
    _rms2[loc] = _rms2[loc + max_size] = nan("");



    if(++size >= max_size) size = max_size;
    if(++loc  >= max_size) loc  = 0;

    for(unsigned i=0; i < slavePv.size(); i++) {
        slavePv[i]->append();    /* execute append() method for all of slaves */
    }

}

void BsaPv::append(unsigned n, double mean, double rms2)
{
    union {
        uint32_t u32;
        int32_t  i32;
        float    f32;
    } u;
    double __mean;

    u.u32 = (uint32_t) mean;



    switch(*_p_type) {
        case int32:
            __mean = (double) u.i32;
            break;
        case uint32:
            __mean = (double) u.u32;
            break;
        case float32:
            __mean = (double) u.f32;
            break;
        default:
           __mean = 0.;
           break;
    }


    _n[loc]    = _n[loc + max_size]    = n;
    _mean[loc] = _mean[loc + max_size] = isnan(mean)? NAN: (*_p_slope * __mean + *_p_offset);
    _rms2[loc] = _rms2[loc + max_size] = isnan(rms2)? NAN: (*_p_slope * sqrt(rms2));

    if(++size >= max_size) size = max_size;
    if(++loc  >= max_size) loc  = 0;

    for(unsigned i=0; i < slavePv.size(); i++) {
        slavePv[i]->append(n, mean, rms2);    /* execute append() method for all of slaves */
        // printf("Slave append %d: slope %lf, offset %lf\n", i, *(slavePv[i]->_p_slope), *(slavePv[i]->_p_offset));
    }
}

void BsaPv::flush()
{
    (_ppvt_num)->ts.secPastEpoch = _ts_sec;
    (_ppvt_num)->ts.nsec         = _ts_nsec;
    (_ppvt_num)->nreq            = size;
    (_ppvt_num)->entry_sz        = sizeof(unsigned);
    (_ppvt_num)->bptr            = &_n[max_size + loc - size];
    pBsaDrv->flush(_ppvt_num);
 
 
    (_ppvt_mean)->ts.secPastEpoch = _ts_sec;
    (_ppvt_mean)->ts.nsec         = _ts_nsec;
    (_ppvt_mean)->nreq            = size;
    (_ppvt_mean)->entry_sz        = sizeof(double);
    (_ppvt_mean)->bptr            = &_mean[max_size + loc - size];
    pBsaDrv->flush(_ppvt_mean);

 
    (_ppvt_rms2)->ts.secPastEpoch = _ts_sec;
    (_ppvt_rms2)->ts.nsec         = _ts_nsec;
    (_ppvt_rms2)->nreq            = size;
    (_ppvt_rms2)->entry_sz        = sizeof(double);
    (_ppvt_rms2)->bptr            = &_rms2[max_size + loc - size];
    pBsaDrv->flush(_ppvt_rms2);


    for(unsigned i=0; i < slavePv.size(); i++) {
        slavePv[i]->flush();              /* execture flush() method for all of slaves */
    }
}




BsaPvArray::BsaPvArray(unsigned array, const std::vector <Bsa::Pv*>& pvs, int p_pid_UL, bsaAsynDriver *pBsaDrv, devBsaPvt_t *ppvt)
                       : _array(array), size(0), loc(0), _pvs(pvs), _p_pid_UL(p_pid_UL), _ppvt_pid(ppvt)
{
    this->pBsaDrv = pBsaDrv;
    max_size = determine_max_size(array);
// reserve emory for better performance
    _pid.reserve(max_size *2); _pid.resize(max_size *2); bsa_item_cnt++;

}


void BsaPvArray::reset(unsigned sec, unsigned nsec)
{

//    printf("BsaPvArray::reset\n");

    _ts_sec = sec;
    _ts_nsec = nsec;

    size = 0;
    loc  = 0;

    for(unsigned i =0; i < _pvs.size(); i++) {
        _pvs[i]->clear();
        _pvs[i]->setTimestamp(_ts_sec, _ts_nsec);
    }

}

void BsaPvArray::set(unsigned sec, unsigned nsec)
{
    _ts_sec = sec;
    _ts_nsec = nsec;

    for(unsigned i = 0; i < _pvs.size(); i++) {
        _pvs[i]->setTimestamp(_ts_sec, _ts_nsec);
    }
}


void BsaPvArray::append(uint64_t pulseId)
{
    _pid[loc] = _pid[loc + max_size] = pulseId;
    if(++size >= max_size) size = max_size;
    if(++loc  >= max_size) loc  = 0;

}

std::vector <Bsa::Pv*> BsaPvArray::pvs()
{
    return std::vector <Bsa::Pv*> (_pvs);
}

void BsaPvArray::flush()
{
    (_ppvt_pid)->ts.secPastEpoch = _ts_sec;
    (_ppvt_pid)->ts.nsec         = _ts_nsec;
    (_ppvt_pid)->nreq            = size;
    (_ppvt_pid)->entry_sz        = sizeof(uint64_t);
    (_ppvt_pid)->bptr            = &_pid[max_size + loc - size];
    pBsaDrv->flush(_ppvt_pid);

}



#ifndef HAVE_YAML
bsaAsynDriver::bsaAsynDriver(const char *portName, const char *ipString, const int num_dyn_param)
    : asynPortDriver(portName,
	                 1,  /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
					 NUM_BSA_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
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
    if(!pBsaEllList || !ellCount(pBsaEllList)) return;

    if(!strcmp(ipString, "SLAVE") || !strcmp(ipString, "slave"))
        pProcessor = Bsa::Processor::create();          // slave mode
    else
        pProcessor = Bsa::Processor::create(ipString);  // master mode

    if(!pProcessor) return;

    _pBsaBridge = (void*) pProcessor->getHardware();

    SetupAsynParams();
    SetupFields();
    SetupPvs();
    SetupPvArray();
}

#else   /* HAVE_YAML */
bsaAsynDriver::bsaAsynDriver(const char *portName, const char *path_reg, const char *path_ram, const int num_dyn_param, ELLLIST *pBsaEllList,  const char *named_root)
    : asynPortDriver(portName,
	                 1,  /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
					 NUM_BSA_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
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
    if(!pBsaEllList || !ellCount(pBsaEllList)) return;

    this->pBsaEllList = pBsaEllList;

    Path root_ = (named_root && strlen(named_root))?cpswGetNamedRoot(named_root):cpswGetRoot();
    if(!root_) {
        printf("BSA driver: could not find root path\n");
        return;
    }
    Path reg_ = root_->findByName(path_reg);
    Path ram_ = root_->findByName(path_ram);

    if(!reg_) {
        printf("BSA driver: could not find register at path %s\n", path_reg);
        return;
    }
    if(!ram_) {
        printf("BSA driver: could not find ram at path %s\n", path_ram);
        return;
    }

    bsa_enable = 1;


    pProcessor = Bsa::Processor::create(reg_, ram_, true);

    if(!pProcessor) {
        printf("BSA driver: could not complete initialization for processor class\n");
        return;
    }

    _pBsaBridge = (void*) pProcessor->getHardware();


    fault_cnt = 0;
    fault = 0;
    pend_err = 0;
    sum_uarray_err = 0;
    for(int i = 0; i < MAX_BSA_ARRAY; i++) {
        uarray_err[i] = 0;
    }


    SetupAsynParams();
    SetupFields();
    SetupPvs();
    SetupPvArray();


}

#endif  /* HAVE_YAML */


bsaAsynDriver::~bsaAsynDriver()
{
}

void bsaAsynDriver::SetupAsynParams(void)
{
    char param_name[80];

    sprintf(param_name, enableString); createParam(param_name, asynParamInt32, &p_enable);
    sprintf(param_name, faultCntString); createParam(param_name, asynParamInt32, &p_fault_cnt); setIntegerParam(p_fault_cnt, fault_cnt);
    sprintf(param_name, statusString);   createParam(param_name, asynParamInt32, &p_status);    setIntegerParam(p_status,    fault);

    for(int i=0; i<MAX_BSA_ARRAY; i++) {
        sprintf(param_name, pidString,  i+START_BSA_ARRAY); createParam(param_name, asynParamInt64Array, &p_pid_UL[i]);

        bsaList_t * p = (bsaList_t *) ellFirst(pBsaEllList);
        while(p) {    /* search for master node */
            sprintf(param_name, numString,  p->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &p->p_num[i]);  strcpy(p->pname_num[i],  param_name);
            sprintf(param_name, meanString, p->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &p->p_mean[i]); strcpy(p->pname_mean[i], param_name);
            sprintf(param_name, rms2String, p->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &p->p_rms2[i]); strcpy(p->pname_rms2[i], param_name);

            bsaList_t * q = (bsaList_t *) ellFirst(p->pSlaveEllList);
            while(q) {  /* search for slave node */
                sprintf(param_name, numString,  q->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &q->p_num[i]);  strcpy(q->pname_num[i],  param_name);
                sprintf(param_name, meanString, q->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &q->p_mean[i]); strcpy(q->pname_mean[i], param_name);
                sprintf(param_name, rms2String, q->bsa_name, i+START_BSA_ARRAY); createParam(param_name, asynParamFloat64Array, &q->p_rms2[i]); strcpy(q->pname_rms2[i], param_name);

                q = (bsaList_t *) ellNext(&q->node);
            }

            p = (bsaList_t *) ellNext(&p->node);
        }

    }

    bsaList_t *p = (bsaList_t *) ellFirst(pBsaEllList);
    while (p) {    /* search for master node */
        sprintf(param_name, slopeString,  p->bsa_name); createParam(param_name, asynParamFloat64, &p->p_slope);  strcpy(p->pname_slope,  param_name);
        sprintf(param_name, offsetString, p->bsa_name); createParam(param_name, asynParamFloat64, &p->p_offset); strcpy(p->pname_offset, param_name);

        bsaList_t *q = (bsaList_t *) ellFirst(p->pSlaveEllList);
        while(q) {    /* search for slave node */
            sprintf(param_name, slopeString,  q->bsa_name); createParam(param_name, asynParamFloat64, &q->p_slope);  strcpy(q->pname_slope,  param_name);
            sprintf(param_name, offsetString, q->bsa_name); createParam(param_name, asynParamFloat64, &q->p_offset); strcpy(q->pname_offset, param_name);
            q = (bsaList_t *) ellNext(&q->node);
        }

        p = (bsaList_t *) ellNext(&p->node);
    }


    callParamCallbacks();
}

devBsaPvt_t * bsaAsynDriver::findPvt(char *key, char *param, int idx)
{
    bsaList_t *p = (bsaList_t *) ellFirst(pBsaEllList);
    int edef = idx - START_BSA_ARRAY;

    if(p && !strcmp(param, PARAM_BSAPID)) return &ppvt_pid[edef];

    while(p) {
        if(!strcmp(key, p->bsa_name)) break;
        p = (bsaList_t *) ellNext(&p->node);
    }

    if(!p) return NULL;

    if(!strcmp(param, PARAM_BSAMEAN)) return &p->ppvt_mean[edef];
    if(!strcmp(param, PARAM_BSANUM))  return &p->ppvt_num[edef];
    if(!strcmp(param, PARAM_BSARMS2)) return &p->ppvt_rms2[edef];

    return NULL;
    
}

void bsaAsynDriver::SetupFields(void)
{
    bsaList_t *p, *q;
    int i;

    for(i = 0; i< MAX_BSA_ARRAY; i++) {
        p = (bsaList_t *) ellFirst(pBsaEllList);
        while(p) {    /* register parent field */
            BsaField *pField = new BsaField(p->bsa_name, i, p->p_num[i], p->p_mean[i], p->p_rms2[i], &p->slope, &p->offset, &p->type, 
                                            &p->ppvt_num[i], &p->ppvt_mean[i], &p->ppvt_rms2[i]);
            fields[i].push_back(pField);

            q = (bsaList_t *) ellFirst(p->pSlaveEllList);
            while(q) {  /* register slave field */
                BsaField *qField = new BsaField(q->bsa_name, i, q->p_num[i], q->p_mean[i], q->p_rms2[i], &q->slope, &q->offset, &p->type,  
                                                &p->ppvt_num[i], &p->ppvt_mean[i], &p->ppvt_rms2[i]);
                pField->slaveField.push_back(qField);
                q = (bsaList_t *) ellNext(&q->node);
            }
            p = (bsaList_t *) ellNext(&p->node);
        }
    }
}

void bsaAsynDriver::SetupPvs(void)
{
    unsigned i, j;

    for(i=0; i< MAX_BSA_ARRAY; i++) {
        for(j=0; j < fields[i].size(); j++) {
            pvs[i].push_back(new BsaPv(*fields[i][j], this));
        }
    }
}

void bsaAsynDriver::SetupPvArray(void)
{

    for(int i=0; i< MAX_BSA_ARRAY; i++) {
        ppvt_pid[i] = {0};
        pBsaPvArray.push_back(new BsaPvArray(i, pvs[i], p_pid_UL[i], this, &ppvt_pid[i]));
    }
}


int bsaAsynDriver::BsaRetreivePoll(void)
{
    epicsTimeStamp    _ts;
    uint64_t  pending;
    unsigned  prev_pend_err       = pend_err;
    unsigned  prev_sum_uarray_err = sum_uarray_err;



        if(bsa_enable) {
            try {
              pending = pProcessor->pending();
            } catch(...) {
                // printf("Bsa Poll: error detecting pProcessor->pending()\n");
                pend_err++;
            }
        }
        else  return 0;

        for(int i=0; i< MAX_BSA_ARRAY; i++) {
            try { 
            if(!(pending & (1ULL << pBsaPvArray[i]->array()))) continue; /* nothing to flush */

            if(pProcessor->update(*pBsaPvArray[i])) {
                // setup timestamp, assumed that the PV flushing should is serializized
                _ts.secPastEpoch  = pBsaPvArray[i]->get_ts_sec();
                _ts.nsec          = pBsaPvArray[i]->get_ts_nsec();
                setTimeStamp(&_ts);

                pBsaPvArray[i]->flush();

                for(unsigned int j = 0; j< pvs[i].size(); j++) {
                    pvs[i][j]->flush();
                }
            }
            } catch(...) {
                // printf("Bsa Poll: error detecting pProcessor->update()\n");
                sum_uarray_err++;
                uarray_err[i]++;
            }

        }

        if((prev_pend_err != pend_err) || (prev_sum_uarray_err != sum_uarray_err)) {
            poll_intv = FAULT_INTV;

            if(!fault) {
                fault = FAULT;
                fault_cnt++;
                setIntegerParam(p_status, fault);
                setIntegerParam(p_fault_cnt, fault_cnt);
                callParamCallbacks();
            }
        } else {
            poll_intv = NORM_INTV;

            if(fault) {
                fault = NORMAL;
                setIntegerParam(p_status, fault);
                callParamCallbacks();
            }
        }

    return 0;
}

int bsaAsynDriver::bsaAsynDriverEnable(void)
{
    bsa_enable = 1;
    return 0;
}

int bsaAsynDriver::bsaAsynDriverDisable(void)
{
    bsa_enable = 0;
    return 0;
}

int bsaAsynDriver::bsaAsynDriverReport(int level)
{

    if(!level) return 0;

    printf("    bsa status: %s,   fault cnt: %d\n", (fault?"FAULT":"NORMAL"), fault_cnt); 
    printf("    pend error: %u,   update error: %u\n", pend_err, sum_uarray_err);
    for(int i = 0; i < MAX_BSA_ARRAY; i++) {
        if(uarray_err[i]) printf("\tarray update error (%d): %u\n", i, uarray_err[i]);
    }

    return 0;
}


asynStatus bsaAsynDriver::flush(devBsaPvt_t *ppvt)
{
    asynStatus status = asynSuccess;

    process_bsa_pv(ppvt);

    return status;
}




asynStatus bsaAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *functionName = "writeInt32";

	/* set the parameter in the parameter library */
	status = (asynStatus) setIntegerParam(function, value);

	switch(function) {
	    default:
		    break;
	}

  if(function == p_enable) {
      if(value) bsaAsynDriverEnable();
      else      bsaAsynDriverDisable();
  }

	/* Do callback so higher layer see any changes */
	status = (asynStatus) callParamCallbacks();

	if(status)
	    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
	                  "%s:%s: status=%d, function=%d, value=%d",
			         		  driverName, functionName, status, function, value);
	else
	    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
		            "%s:%s: function=%d, value=%d\n",
				        driverName, functionName, function, value);

    return status;
}

asynStatus bsaAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    const char *functionName = "writeFloat64";

    /* set the parameter in the parameter library */
    status = (asynStatus) setDoubleParam(function, value);


    bsaList_t * p = (bsaList_t *) ellFirst(pBsaEllList);
    while(p) {           // update slope and offset for master BSA node
        if(function == p->p_slope) {
            p->slope = value;
            // printf("%s:%s slope %lf\n", p->bsa_name, p->bsa_type, value);
        }
        if(function == p->p_offset) {
            p->offset = value;
            // printf("%s:%s offset %lf\n", p->bsa_name, p->bsa_type,value);
        }


        bsaList_t *q = (bsaList_t *) ellFirst(p->pSlaveEllList);
        while(q) {    // update slope and offset for slave BSA node
            if(function == q->p_slope) {
                q->slope = value;
                // printf("%s:%s slope %lf\n", q->bsa_name, q->bsa_type, value);
            }

            if(function == q->p_offset) {
                q->offset = value;
                // printf("%s:%s slope %lf\n", q->bsa_name, q->bsa_type, value);
            }
            q = (bsaList_t *) ellNext(&q->node);
        }

        p = (bsaList_t *) ellNext(&p->node);
    }

    /* Do callback so higher layer see any changes */
    status = (asynStatus) callParamCallbacks();

    if(status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:%s status=%d, function=%d, value=%lf",
                      driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%lf\n",
                  driverName, functionName, function, value);


    return status;
}



extern "C" {

#ifndef HAVE_YAML

int bsaAsynDriverConfigure(const char *portName, const char *ipString)
{
    if(!pBsaEllList) return -1;

    int i = 0;
    bsaList_t *p = (bsaList_t *) ellFirst(pBsaEllList);
    while(p) {
        i += (int)(&p->lastParam - &p->firstParam -1);
        p = (bsaList_t *) ellNext(&p->node);
    }  /* calculate total number of dynamic parameters */

    pBsaDrv = new bsaAsynDriver(portName, ipString, i);
    strcpy(port_name, portName);
    strcpy(ip_string, ipString);
    strcpy(reg_path_string, "not applicable");
    strcpy(ram_path_string, "not applicable");
    return 0;
}

#else /* HAVE_YAML */

int bsaAsynDriverConfigure(const char *portName, const char *regPathString, const char *ramPathString, const char *named_root)
{
    pDrvList_t *pl = find_drvByPort(portName);
    if(!pl) {
         pl = find_drvLast();
         if(pl) {
             if(!pl->port && !pl->named_root && !pl->pBsaDrv) pl->port = epicsStrDup(portName);
             else pl = NULL;
         }
    }
    if(!pl) {
        printf("Bsa list never been configured for port (%s)\n", portName);
        return -1;
    }

    pl->named_root = (named_root && strlen(named_root))?epicsStrDup(named_root):cpswGetRootName();
    
    if(!pl->pBsaEllList) return -1;

    int i = 0;
    bsaList_t *p, *q;
    p = (bsaList_t *) ellFirst(pl->pBsaEllList);
    while(p) {                                           /* search master node */
        i += (int)(&p->lastParam - &p->firstParam -1);
        q = (bsaList_t *) ellFirst(p->pSlaveEllList);
        while (q) {                                      /* search slave node */
            i += (int)(&q->lastParam - &q->firstParam-1);
            q = (bsaList_t *) ellNext(&q->node);
        }
        p = (bsaList_t *) ellNext(&p->node);
    }  /* calculate total number of dynamic parameters */

    pl->pBsaDrv = new bsaAsynDriver(portName, regPathString, ramPathString, i, pl->pBsaEllList, pl->named_root);
    strcpy(port_name, portName);
    strcpy(ip_string, "not applicable");
    strcpy(reg_path_string, regPathString);
    strcpy(ram_path_string, ramPathString);
    return 0;
}

int bsaAsynDriverEnable(const char *idString)
{
    pDrvList_t *p = NULL;

    if(idString) {
        p = find_drvByPort(idString);
        if(!p) p = find_drvByNamedRoot(idString);
    }
    else  p = find_drvLast();

    if(p && p->pBsaDrv) { 
        p->pBsaDrv->bsaAsynDriverEnable();
        return 0;
    }


    printf("failt to enable BSA due to the bsaAsynDriver never been initialized.\n");
    return 0;
}

int bsaAsynDriverDisable(const char *idString)
{
    pDrvList_t *p = NULL;

    if(idString) {
        p = find_drvByPort(idString);
        if(!p) p = find_drvByNamedRoot(idString);
    }
    else p = find_drvLast();

    if(p && p->pBsaDrv) {
        p->pBsaDrv->bsaAsynDriverDisable();
        return 0;
    }


    printf("fail to disable BSA due to the bsaAsynDriver never been initialized.\n");
    return 0;
}


#endif  /* HAVE_YAML */

static bsaList_t * findBsa(ELLLIST *pBsaEllList, const char *bsaKey);

bsaDataType_t getBsaDataType(const char *bsaType)
{
    bsaDataType_t type;

    if(!strcmp(INT32STRING,        bsaType)) type = int32;
    else if(!strcmp(UINT32STRING,  bsaType)) type = uint32;
    else if(!strcmp(UINT64STRING,  bsaType)) type = uint64;
    else if(!strcmp(FLOAT32STRING, bsaType)) type = float32;
    else type = fault;

    return type;

}

int createBsaList(const char *port_name)
{
    pDrvList_t *pl = find_drvByPort(port_name);

    if(pl) {
        printf("Bsa List (port: %s) already exists\n", port_name);
        return 0;
    }

    prep_drvByPort(port_name);

    return 0;
}

int bsaAdd(const char *bsaKey, const char *bsaType, const char *keepAsInteger)
{
    pDrvList_t *pl = find_drvLast();

    if(pl) {
        if(pl->pBsaDrv ||
           (pl->port && strlen(pl->port))) pl = NULL;   /* the driver node has been configured, need to make another one */
    }

    while(!pl) {
        prep_drvAnonimous();
        pl = find_drvLast();
    }


    bsaList_t *p = (bsaList_t *) mallocMustSucceed(sizeof(bsaList_t), "bsaDriver (bsaAdd)");
    int i;
 

    if(!pl->pBsaEllList) {    /* initialize for once */
         pl->pBsaEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsaDriver(bsaAdd)");
        ellInit(pl->pBsaEllList);
    }

    p->pSlaveEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "bsaDriver(bsaAdd)");
    ellInit(p->pSlaveEllList);        //init. linked list for slave

    strcpy(p->bsa_name, bsaKey);
    strcpy(p->bsa_type, bsaType);

    if (keepAsInteger == NULL || (strcmp("0", keepAsInteger) == 0))
        p->doNotTouch = false;
    else
        p->doNotTouch = true;
    
    for(i=0; i< MAX_BSA_ARRAY; i++) {
        p->p_num[i]        = p->p_mean[i]        = p->p_rms2[i]        = -1;   // intialize to invalid parameter
        p->pname_num[i][0] = p->pname_mean[i][0] = p->pname_rms2[i][0] = '\0'; // make null string
        p->ppvt_num[i] = {0};
        p->ppvt_mean[i] = {0};
        p->ppvt_rms2[i] = {0};
    }

    p->slope  = 1.;   // setup default linear conversion, it will be override by epics PV
    p->offset = 0.;   // setup drfault linear conversion, it will be override by epics PV

    p->type = getBsaDataType(p->bsa_type);

    if(p->type == fault) {
        printf("Error in bsaAdd(): could not add %s due to wrong type descriptor (%s)\n", bsaKey, bsaType);
        return 0;
    }

    ellAdd(pl->pBsaEllList, &p->node);

    /* add Bsa */

    return 0;
}

int addSlaveBsa(const char *bsaKey, const char *slaveKey, const char *bsaType)
{

    pDrvList_t *pl = find_drvLast();
    while(!pl) {
        prep_drvAnonimous();
        pl = find_drvLast();
    }


    bsaList_t *p, *q;
    int i;

    p = findBsa(pl->pBsaEllList, bsaKey);
    if(!p) {
        printf("Could not find bsa node (%s)\n", bsaKey);
        return -1;
    }
    q = (bsaList_t *) mallocMustSucceed(sizeof(bsaList_t), "bsaDriver(addSlaveBsa)");
    if(!q) {
        printf("memory allocation error\n");
        return -1;
    }
    q->pSlaveEllList = (ELLLIST *) NULL;

    strcpy(q->bsa_name, slaveKey);
    strcpy(q->bsa_type, bsaType);

    for(i = 0; i <MAX_BSA_ARRAY; i++) {
        q->p_num[i]        = q->p_mean[i]        = q->p_rms2[i]         = -1;       // initialize to invalid parameter
        q->pname_num[i][0] = q->pname_mean[i][0] = q->pname_rms2[i][0] = '\0';      // make nulll string
    }

    q->slope  = 1.;
    q->offset = 0.;

    p->type = getBsaDataType(p->bsa_type);

    if(p->type == fault) {
        printf("Error in addSlaveBsa(): could not add %s into %s due to wrong type descriptor (%s)\n", slaveKey, bsaKey, bsaType);
        return 0;
    }

    ellAdd(p->pSlaveEllList, &q->node);

    return 0;
}


static int _bsaList(ELLLIST *pBsaEllList)
{
    bsaList_t *p, *q;
    int       i = 0;
    int       j;

    if(!pBsaEllList) return -1;
    printf("Total %d BSA(s) has(have) been registered\n", ellCount(pBsaEllList));

    p = (bsaList_t *) ellFirst(pBsaEllList);
    while (p) {
        printf("\t%d\t%s, %s\n", i++, p->bsa_name, p->bsa_type);  // print out for master node
        q = (bsaList_t *) ellFirst(p->pSlaveEllList);
        j = 0;
        while(q) {
            printf("\t\t+----- slave node %d\t%s, %s\n", j++, q->bsa_name, q->bsa_type);    // print out for slave nodes
            q = (bsaList_t *) ellNext(&q->node);
        }
        p = (bsaList_t *) ellNext(&p->node);
    }

    return 0;
}


static int _bsaListAll(void)
{
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        printf("Bsa Driver (named_root: %s, port: %s)\n",
               (p->named_root)?p->named_root: "Unknown", (p->port)?p->port: "Unknown");
        if(p->pBsaEllList) _bsaList(p->pBsaEllList);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}


static int _bsaListByName(const char *name)
{

    pDrvList_t *p = find_drvByPort(name);
    if(!p)      p = find_drvByNamedRoot(name);
    if(!p) {
        printf("Could not find BSA list for %s\n", name);
        return 0;
    }

    printf("Bsa Driver (named_root: %s, port: %s)\n",
           (p->named_root)?p->named_root: "Unknown", (p->port)?p->port: "Unknown");
    if(p->pBsaEllList) _bsaList(p->pBsaEllList);

    return 0;
}



int bsaList(const char *name = NULL)
{
    if(name && strlen(name)) return _bsaListByName(name);
    else                      return _bsaListAll();
}

static bsaList_t * findBsa(ELLLIST *pBsaEllList, const char *bsaKey)
{
    bsaList_t *p;

    if(!pBsaEllList || !ellCount(pBsaEllList)) return (bsaList_t *) NULL;

    p = (bsaList_t *) ellFirst(pBsaEllList);
    while(p) {
        if(!strcmp(bsaKey, p->bsa_name)) break;
        p = (bsaList_t *) ellNext(&p->node);
    }

    return p;
}

// interface for BSSS driver
ELLLIST * find_bsaChannelList(const char *port_name)
{
    pDrvList_t *p = find_drvByPort(port_name);
    
    if(p) return p->pBsaEllList;
    else  return NULL;
}


// inteface for BSA device support
devBsaPvt_t * find_bsa_pvt(char *port, char *key, char *param, int edef)
{
    pDrvList_t *pDrv = find_drvByPort(port);

    if(pDrv && pDrv->pBsaDrv) return pDrv->pBsaDrv->findPvt(key, param, edef); 
    else                      return NULL;
}


/* EPICS iocsh shell commands */


#ifndef HAVE_YAML

static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "ipAddress", iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1 };
static const iocshFuncDef initFuncDef = { "bsaAsynDriverConfigure", 2, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{

    bsaAsynDriverConfigure(args[0].sval, args[1].sval);
}

#else   /* HAVE_YAML */

static const iocshArg initArg0 = { "portName",                                           iocshArgString };
static const iocshArg initArg1 = { "register path (which should be described in yaml):", iocshArgString };
static const iocshArg initArg2 = { "ram path (which should be described in yaml):",      iocshArgString };
static const iocshArg initArg3 = { "named_root (optional)",                              iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3 };
static const iocshFuncDef initFuncDef = { "bsaAsynDriverConfigure", 4, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
 
    bsaAsynDriverConfigure(args[0].sval,  /* port name */
                           args[1].sval,  /* register path */
                           args[2].sval, /* ram path */
                           (args[3].sval && strlen(args[3].sval))? args[3].sval: NULL);  /* named_root */
}

#endif  /* HAVE_YAML */

static const iocshArg enableArg0 = {"portName or named_root (optional)", iocshArgString };
static const iocshArg * const enableArgs[] = { &enableArg0 };
static const iocshFuncDef bsaEnableFuncDef  = { "bsaAsynDriverEnable",  1, enableArgs};
static void bsaEnableCallFunc(const iocshArgBuf *args)
{
    bsaAsynDriverEnable( (args[0].sval && strlen(args[0].sval))? args[0].sval: NULL);
}

static const iocshArg disableArg0 = {"portName or named_string (optional)", iocshArgString };
static const iocshArg * const disableArgs [] = { &disableArg0 };
static const iocshFuncDef bsaDisableFuncDef = { "bsaAsynDriverDisable", 1, disableArgs };
static void bsaDisableCallFunc(const iocshArgBuf *args)
{
    bsaAsynDriverDisable((args[0].sval && strlen(args[0].sval))? args[0].sval: NULL);
}


static const iocshArg createArg0 = { "port name", iocshArgString };
static const iocshArg * const createArgs [] = { &createArg0 };
static const iocshFuncDef createBsaFuncDef = {"createBsaList", 1, createArgs};
static void createBsaCallFunc(const iocshArgBuf *args)
{
    createBsaList((args[0].sval && strlen(args[0].sval))? args[0].sval: NULL);
}


static const iocshArg bsaAddArg0 = { "bsaKey", iocshArgString };
static const iocshArg bsaAddArg1 = { "bsaType", iocshArgString };
static const iocshArg bsaAddArg2 = { "keepAsInteger (optional)",iocshArgString };
static const iocshArg * const bsaAddArgs [] = { &bsaAddArg0,
                                                &bsaAddArg1,
                                                &bsaAddArg2 };
static const iocshFuncDef bsaAddFuncDef = { "bsaAdd", 3, bsaAddArgs };
static void bsaAddCallFunc(const iocshArgBuf *args)
{

    bsaAdd(args[0].sval, args[1].sval, (args[2].sval && strlen(args[2].sval))? args[2].sval: NULL);
}

static const iocshArg addSlaveBsaArg0 = {"bsaKey",   iocshArgString};
static const iocshArg addSlaveBsaArg1 = {"slaveKey", iocshArgString};
static const iocshArg addSlaveBsaArg2 = {"bsaType",  iocshArgString};
static const iocshArg * const addSlaveBsaArgs [] = { &addSlaveBsaArg0,
                                                     &addSlaveBsaArg1,
                                                     &addSlaveBsaArg2 };
static const iocshFuncDef addSlaveBsaFuncDef = {"addSlaveBsa", 3, addSlaveBsaArgs};
static void addSlaveBsaCallFunc(const iocshArgBuf *args)
{
    addSlaveBsa(args[0].sval, args[1].sval, args[2].sval);
}


static const iocshArg bsaListArg0 = {"named_root or port (optional)", iocshArgString};
static const iocshArg * const bsaListArgs[] = { &bsaListArg0 };
static const iocshFuncDef bsaListFuncDef = { "bsaList", 1, bsaListArgs };
static void bsaListCallFunc(const iocshArgBuf *args)
{
    bsaList((args[0].sval && strlen(args[0].sval))? args[0].sval: NULL);
}

void bsaAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,        initCallFunc);
    iocshRegister(&bsaEnableFuncDef,   bsaEnableCallFunc);
    iocshRegister(&bsaDisableFuncDef,  bsaDisableCallFunc);
    iocshRegister(&createBsaFuncDef,   createBsaCallFunc);
    iocshRegister(&bsaAddFuncDef,      bsaAddCallFunc);
    iocshRegister(&addSlaveBsaFuncDef, addSlaveBsaCallFunc);
    iocshRegister(&bsaListFuncDef,     bsaListCallFunc);
}

epicsExportRegistrar(bsaAsynDriverRegister);



static int BsaRetreivePoll(void)
{
    while(1) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pBsaDrv) p->pBsaDrv->BsaRetreivePoll();
            p = (pDrvList_t *) ellNext(&p->node);
        }
        epicsThreadSleep(poll_intv);
    }

    return 0;
}


/* EPICS driver support for bsaAsynDriver */

static int bsaAsynDriverReport(int interest);
static int bsaAsynDriverInitialize(void);

static struct drvet bsaAsynDriver = {
    2,
	(DRVSUPFUN) bsaAsynDriverReport,
	(DRVSUPFUN) bsaAsynDriverInitialize
};

epicsExportAddress(drvet, bsaAsynDriver);

static int bsaAsynDriverReport(int interest)
{
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    if(!p) {
        printf("No BSA driver instances\n");
        return 0;
    }

    while(p) {
        printf("named_root: %s, port: %s, driver instace: %p, number of BSA varibles: %d\n",
              (p->named_root && strlen(p->named_root))?p->named_root: "Unknown",
              (p->port && strlen(p->port))? p->port: "Unknown",
              p->pBsaDrv,
              (p->pBsaEllList)? ellCount(p->pBsaEllList): -1);

        if(p->pBsaDrv) p->pBsaDrv->bsaAsynDriverReport(interest);

        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int bsaAsynDriverInitialize(void)
{

   /* Implement EPICS driver initialization here */
    init_drvList();

    if(!pDrvEllList) {
        printf("Bsa Driver never been configured\n");
        return 0;
    }

    epicsThreadCreate("bsaDrvPoll", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) BsaRetreivePoll, 0);

	return 0;
}

} /* extern "C" */

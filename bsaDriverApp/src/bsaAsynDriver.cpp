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

static const  char *driverName = "bsaAsynDriver";
static char port_name[32];
static char ip_string[32];
static char reg_path_string[256];
static char ram_path_string[256];

static int  once = 0;      /* do not make more than one instace */
static int  addBsa_once = 0;
static class bsaAsynDriver *pBsaDrv = NULL;

static ELLLIST *pBsaEllList = NULL;

BsaField::BsaField(char *name, int index, int p_num, int p_mean, int p_rms2)
{
    std::ostringstream o;
    o << name << index;
    _name = o.str();
    
    _p_num  = p_num;
    _p_mean = p_mean;
    _p_rms2 = p_rms2;
}


BsaPv::BsaPv (Bsa::Field& f) : _f(f), _n(0), _mean(0), _rms2(0), size(0), loc(0)
{
    BsaField *p = (BsaField *)&_f;

    _p_num  = p->get_p_num();
    _p_mean = p->get_p_mean();
    _p_rms2 = p->get_p_rms2();
    
// reserve memory for better performance
// need to test to measure improvement   
   _n.reserve(MAX_BSA_LENGTH *2);     _n.resize(MAX_BSA_LENGTH *2);
   _mean.reserve(MAX_BSA_LENGTH *2);  _mean.resize(MAX_BSA_LENGTH *2);
   _rms2.reserve(MAX_BSA_LENGTH *2);  _rms2.resize(MAX_BSA_LENGTH *2);
}





const char* BsaPv::name()
{
    return _f.name();
}

void BsaPv::clear()
{
    size = 0;
    loc  = 0;
}


void BsaPv::setTimestamp(unsigned sec, unsigned nsec)
{
    _ts_sec  = sec;
    _ts_nsec = nsec;
}

void BsaPv::append()
{

    _n[loc]    = _n[loc + MAX_BSA_LENGTH] = 0;
    _mean[loc] = _mean[loc + MAX_BSA_LENGTH] = nan("");
    _rms2[loc] = _rms2[loc + MAX_BSA_LENGTH] = nan("");
    
    
    
    if(++size >= MAX_BSA_LENGTH) size = MAX_BSA_LENGTH;
    if(++loc  >= MAX_BSA_LENGTH) loc  = 0;

}

void BsaPv::append(unsigned n, double mean, double rms2)
{
    _n[loc]    = _n[loc + MAX_BSA_LENGTH]    = n;
    _mean[loc] = _mean[loc + MAX_BSA_LENGTH] = mean;
    _rms2[loc] = _rms2[loc + MAX_BSA_LENGTH] = rms2;
    
    if(++size >= MAX_BSA_LENGTH) size = MAX_BSA_LENGTH;
    if(++loc  >= MAX_BSA_LENGTH) loc  = 0;
}

void BsaPv::flush()
{
// need to callback to asyn level to post out PVs
    pBsaDrv->flush(&_n[MAX_BSA_LENGTH + loc -size],   size, _p_num);
    pBsaDrv->flush(&_mean[MAX_BSA_LENGTH + loc-size], size, _p_mean);
    pBsaDrv->flush(&_rms2[MAX_BSA_LENGTH + loc-size], size, _p_rms2);
}




BsaPvArray::BsaPvArray(unsigned array, const std::vector <Bsa::Pv*>& pvs, int p_pid_U, int p_pid_L) 
                       : _array(array), size(0), loc(0), _pvs(pvs), _p_pid_U(p_pid_U), _p_pid_L(p_pid_L)
{ 
// reserve emory for better performance
    _pid.reserve(MAX_BSA_LENGTH *2); _pid.resize(MAX_BSA_LENGTH *2);   
    
    _pidU.reserve(MAX_BSA_LENGTH);
    _pidL.reserve(MAX_BSA_LENGTH);
}


void BsaPvArray::reset(unsigned sec, unsigned nsec)
{

    printf("BsaPvArray::reset\n");
    
    _ts_sec = sec;
    _ts_nsec = nsec;
    
    size = 0;
    loc  = 0;
    
    _pidU.clear();
    _pidL.clear();
    
    for(unsigned i =0; i < _pvs.size(); i++) {
        _pvs[i]->clear();
        _pvs[i]->setTimestamp(_ts_sec, _ts_nsec);
    }
    
}


void BsaPvArray::append(uint64_t pulseId) 
{
    _pid[loc] = _pid[loc + MAX_BSA_LENGTH] = pulseId;
    if(++size >= MAX_BSA_LENGTH) size = MAX_BSA_LENGTH;
    if(++loc  >= MAX_BSA_LENGTH) loc  = 0;
    
}

std::vector <Bsa::Pv*> BsaPvArray::pvs()
{
    return std::vector <Bsa::Pv*> (_pvs);
}

void BsaPvArray::flush()
{
    _pidU.clear();
    _pidL.clear();
    
    for(unsigned int i=0; i < size; i++) {
        _pidU.push_back(unsigned(_pid[MAX_BSA_LENGTH + loc - size + i] >> 32));
        _pidL.push_back(unsigned(_pid[MAX_BSA_LENGTH + loc - size + i]));
    
    }
    
    pBsaDrv->flush(_pidU.data(), size, _p_pid_U);
    pBsaDrv->flush(_pidL.data(), size, _p_pid_L);
}



#ifndef HAVE_YAML
bsaAsynDriver::bsaAsynDriver(const char *portName, const char *ipString, const int num_dyn_param)
    : asynPortDriver(portName,
	                 1,  /* number of elements of this device */
					 NUM_BSA_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
					 asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt32ArrayMask | asynFloat64ArrayMask,    /* Interface mask */
					 asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask | asynInt32ArrayMask | asynFloat64ArrayMask,       /* Interrupt mask */
					 1,    /* asynFlags. This driver does block and it is non multi-device, so flag is 1. */
					 1,    /* Auto connect */
					 0,    /* Default priority */
					 0)    /* Default stack size */
{
    if(once) return;
   	once = 1;
    if(!pBsaEllList || !ellCount(pBsaEllList)) return;
    
    if(!strcmp(ipString, "SLAVE") || !strcmp(ipString, "slave")) 
        pProcessor = Bsa::Processor::create();          // slave mode
    else 
        pProcessor = Bsa::Processor::create(ipString);  // master mode
    
    if(!pProcessor) return;
    
    SetupAsynParams();
    SetupFields();
    SetupPvs();
    SetupPvArray();
}

#else   /* HAVE_YAML */
bsaAsynDriver::bsaAsynDriver(const char *portName, const char *path_reg, const char *path_ram, const int num_dyn_param)
    : asynPortDriver(portName,
	                 1,  /* number of elements of this device */
					 NUM_BSA_DET_PARAMS +  num_dyn_param ,    /* number of asyn params to be cleared for each device */
					 asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt32ArrayMask | asynFloat64ArrayMask,    /* Interface mask */
					 asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask | asynInt32ArrayMask | asynFloat64ArrayMask,       /* Interrupt mask */
					 1,    /* asynFlags. This driver does block and it is non multi-device, so flag is 1. */
					 1,    /* Auto connect */
					 0,    /* Default priority */
					 0)    /* Default stack size */
{
    if(once) return;
   	once = 1;
    if(!pBsaEllList || !ellCount(pBsaEllList)) return;
    
    Path root_ = cpswGetRoot();
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
    
   
    pProcessor = Bsa::Processor::create(reg_, ram_, true);
    
    if(!pProcessor) {
        printf("BSA driver: could not complete initialization for processor class\n"); 
        return;
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
    char param_name[64];
    
    for(int i=0; i<MAX_BSA_ARRAY; i++) {
        sprintf(param_name, pidUString, i); createParam(param_name, asynParamInt32Array, &p_pid_U[i]);
        sprintf(param_name, pidLString, i); createParam(param_name, asynParamInt32Array, &p_pid_L[i]);
        
        bsaList_t * p = (bsaList_t *) ellFirst(pBsaEllList);
        while(p) {
            sprintf(param_name, numString,  p->bsa_name, i); createParam(param_name, asynParamFloat64Array, &p->p_num[i]);  strcpy(p->pname_num[i],  param_name);
            sprintf(param_name, meanString, p->bsa_name, i); createParam(param_name, asynParamFloat64Array, &p->p_mean[i]); strcpy(p->pname_mean[i], param_name);
            sprintf(param_name, rms2String, p->bsa_name, i); createParam(param_name, asynParamFloat64Array, &p->p_rms2[i]); strcpy(p->pname_rms2[i], param_name);
            p = (bsaList_t *) ellNext(&p->node);
        }
    
    }
    
    
}


void bsaAsynDriver::SetupFields(void)
{
    bsaList_t *p;
    int i;
    
    for(i = 0; i< MAX_BSA_ARRAY; i++) {
        p = (bsaList_t *) ellFirst(pBsaEllList);
        while(p) {
            fields[i].push_back(new BsaField(p->bsa_name, i, p->p_num[i], p->p_mean[i], p->p_rms2[i]));
            p = (bsaList_t *) ellNext(&p->node);
        }
    }
}

void bsaAsynDriver::SetupPvs(void)
{
    unsigned i, j;
    
    for(i=0; i< MAX_BSA_ARRAY; i++) {
        for(j=0; j < fields[i].size(); j++) {
            pvs[i].push_back(new BsaPv(*fields[i][j]));
        }
    }
}

void bsaAsynDriver::SetupPvArray(void)
{
    
    for(int i=0; i< MAX_BSA_ARRAY; i++) pBsaPvArray.push_back(new BsaPvArray(i, pvs[i], p_pid_U[i], p_pid_L[i]));
}


int bsaAsynDriver::BsaRetreivePoll(void)
{
    epicsTimeStamp    _ts;
    uint64_t  pending;
    
    while(1) {
        pending = pProcessor->pending();
        
        for(int i=0; i< MAX_BSA_ARRAY; i++) {
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
        
        }
        epicsThreadSleep(.1);
    }



    return 0;
}

asynStatus bsaAsynDriver::flush(double *pData, unsigned size, int param)
{
    asynStatus status;
    
    status = doCallbacksFloat64Array((epicsFloat64*) pData, size, param, 0);
    
    return status;
}


asynStatus bsaAsynDriver::flush(unsigned *pData, unsigned size, int param)
{
    asynStatus status;
    
    status = doCallbacksInt32Array((epicsInt32*) pData, size, param, 0);
    
    return status;
}

asynStatus bsaAsynDriver::flush(int *pData, unsigned size, int param)
{
    asynStatus status;
    
    status = doCallbacksInt32Array((epicsInt32*) pData, size, param, 0);
    
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

int bsaAsynDriverConfigure(const char *portName, const char *regPathString, const char *ramPathString)
{
    if(!pBsaEllList) return -1;
    
    int i = 0;
    bsaList_t *p = (bsaList_t *) ellFirst(pBsaEllList);
    while(p) {
        i += (int)(&p->lastParam - &p->firstParam -1);
        p = (bsaList_t *) ellNext(&p->node);
    }  /* calculate total number of dynamic parameters */

    pBsaDrv = new bsaAsynDriver(portName, regPathString, ramPathString, i);
    strcpy(port_name, portName);
    strcpy(ip_string, "not applicable");
    strcpy(reg_path_string, regPathString);
    strcpy(ram_path_string, ramPathString);
    return 0;
}


#endif  /* HAVE_YAML */


/* EPICS iocsh shell commands */


#ifndef HAVE_YAML

static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "ipAddress", iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0, 
                                             &initArg1 };
static const iocshFuncDef initFuncDef = { "bsaAsynDriverConfigure", 2, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    if(once) {
	    printf("The BSA driver allows single instance."
		       "Duplicated configuration is ignored.\n");
		return;
	}
	
	bsaAsynDriverConfigure(args[0].sval, args[1].sval);
}

#else   /* HAVE_YAML */

static const iocshArg initArg0 = { "portName",                                           iocshArgString };
static const iocshArg initArg1 = { "register path (which should be described in yaml):", iocshArgString };
static const iocshArg initArg2 = { "ram path (which should be described in yaml):",      iocshArgString };
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = { "bsaAsynDriverConfigure", 3, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    if(once) {
        printf("The BSA driver allows single instance."
               "Duplicated configuration is ignored.\n"
               "This constraint is only for prototype driver,"
               "we are going to allow multiple instance in near future.\n");
        return; 
    }
    
    bsaAsynDriverConfigure(args[0].sval,  /* port name */
                           args[1].sval,  /* register path */
                           args[2].sval); /* ram path */ 
}

#endif  /* HAVE_YAML */


static const iocshArg addBsaArg0 = { "bsaName", iocshArgString };
static const iocshArg addBsaArg1 = { "bsaType", iocshArgString };
static const iocshArg * const addBsaArgs [] = { &addBsaArg0,
                                                &addBsaArg1 };
static const iocshFuncDef addBsaFuncDef = { "addBsa", 2, addBsaArgs };
static void addBsaCallFunc(const iocshArgBuf *args)
{
    bsaList_t *p = (bsaList_t *) malloc(sizeof(bsaList_t));
    int i;
    if(!p) {
        printf("memory allocation error\b");
        return;
    }
    
    if(!addBsa_once) {    /* initialize for once */
        addBsa_once = 1;
        
        pBsaEllList = (ELLLIST *) malloc(sizeof(ELLLIST));
        ellInit(pBsaEllList);        
    }
    
    strcpy(p->bsa_name, args[0].sval);
    strcpy(p->bsa_type, "double");    //temporally hardcoded 
    // strcpy(p->bsa_type, args[1].sval);
    
    for(i=0; i< MAX_BSA_ARRAY; i++) {
        p->p_num[i]        = p->p_mean[i]        = p->p_rms2[i]        = -1;   // intialize to invalid parameter
        p->pname_num[i][0] = p->pname_mean[i][0] = p->pname_rms2[i][0] = '\0'; // make null string
    }
    
    ellAdd(pBsaEllList, &p->node);
    
    /* add Bsa */
}


static const iocshArg listBsaArg0 = { "bsaName", iocshArgString };  // not required now, but prepare for future
static const iocshArg listBsaArg1 = { "bsaType", iocshArgString };  // not required now, but prepare for future
static const iocshArg * const listBsaArgs [] = { &listBsaArg0,
                                                 &listBsaArg1 };
static const iocshFuncDef listBsaFuncDef = { "listBsa", 2, listBsaArgs };
static void listBsaCallFunc(const iocshArgBuf *args)
{
    bsaList_t *p;
    int       i = 0;

    if(!pBsaEllList) return;
    printf("Total %d BSA(s) has(have) been registered\n", ellCount(pBsaEllList));
    
    p = (bsaList_t *) ellFirst(pBsaEllList);
    while (p) {
        printf("\t%d\t%s, %s\n", i++, p->bsa_name, p->bsa_type);
        p = (bsaList_t *) ellNext(&p->node);
    }
}

void bsaAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,    initCallFunc);
    iocshRegister(&addBsaFuncDef,  addBsaCallFunc);
    iocshRegister(&listBsaFuncDef, listBsaCallFunc);
}

epicsExportRegistrar(bsaAsynDriverRegister);



static int BsaRetreivePoll(void)
{
    return pBsaDrv->BsaRetreivePoll();
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
    if(!once) return 0;
	
	/* Implement report function here */
	
	return 0;
}

static int bsaAsynDriverInitialize(void)
{
    if(!once) printf("Driver Initialization is so early.\n");
	  else      printf("Driver Initialization is good.\n");
	
	/* Implement EPICS driver initialization here */
 
     if(!pBsaDrv) return -1;
 
    epicsThreadCreate("bsaDrvPoll", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) BsaRetreivePoll, 0);
 
	return 0;
}

} /* extern "C" */

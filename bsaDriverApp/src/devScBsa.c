#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "alarm.h"
#include "cvtTable.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "dbScan.h"

#include "recGbl.h"
#include "recSup.h"
#include "devSup.h"
#include "link.h"
#include "epicsExport.h"

#include "waveformRecord.h"

#include "devScBsa.h"



typedef struct {
    void       *pvt;
    IOSCANPVT  ioscanpvt;
} dev_pvt;


void process_bsa_pv(devBsaPvt_t *p)
{
    dev_pvt *dpvt = (dev_pvt *) p->dpvt;
    if(dpvt && dpvt->ioscanpvt) scanIoRequest(dpvt->ioscanpvt);
}


static long get_ioint_info(int cmd, dbCommon *prec, IOSCANPVT *ppvt)
{
    dev_pvt *dpvt = (dev_pvt *) prec->dpvt;

    if(dpvt){
        if(dpvt->ioscanpvt == 0) scanIoInit(&dpvt->ioscanpvt);
        *ppvt = dpvt->ioscanpvt;
    } else {
        *ppvt = 0;
    }


    return 0;
}



static long init_record(waveformRecord *prec)
{
    char port[32], key[32], param[32];
    int  edef;

    devBsaPvt_t *ppvt;
    dev_pvt *dpvt = (dev_pvt *) malloc(sizeof(dev_pvt));
    memset(dpvt, 0, sizeof(dev_pvt));

    switch(prec->inp.type) {
        case INST_IO:
            sscanf(prec->inp.value.instio.string, "%s %s %s %d", port, key, param, &edef);
            ppvt = find_bsa_pvt(port, key, param, edef);
            if(!ppvt) goto err;
            dpvt->pvt = (void *) ppvt;
            ppvt->dpvt = (void *) dpvt;
            if(prec->bptr) free(prec->bptr);  // since we are trying zero copy buffer, 
                                              // we do not need pre-allocated memory in the record
            break;
        default:
            err:
            recGblRecordError(S_db_badField, (void *) prec,
                              "devScBsa (init_record) Illegal INP field");
            free(dpvt);
            prec->udf = TRUE;
            prec->dpvt = NULL;
            return S_db_badField;
    }

    prec->udf = FALSE;
    prec->dpvt = (void *) dpvt;
    return 0;
}

static long read_wf(waveformRecord *prec)
{
    int nreq;
    devBsaPvt_t *ppvt;
    dev_pvt     *dpvt = (dev_pvt *) prec->dpvt;
    if(!dpvt) return 0;

    ppvt = dpvt->pvt;
    prec->time = ppvt->ts;
    
    nreq = ppvt->nreq;
    if(nreq > prec->nelm) nreq = prec->nelm;
    if(nreq != prec->nord) prec->nord = nreq;
    // memcpy(prec->bptr, ppvt->bptr, nreq * ppvt->entry_sz);
    prec->bptr = ppvt->bptr;    // using zerro  copy buffer, 
                                // simply expose driver layer buffer to record

    

    return 0;
}


struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_wf;
} devScBsa = {
    5,
    NULL,
    NULL,
    init_record,
    get_ioint_info,
    read_wf
};
epicsExportAddress(dset, devScBsa);


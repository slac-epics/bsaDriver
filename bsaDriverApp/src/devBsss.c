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

#include "aiRecord.h"
#include "int64inRecord.h"

#include "devBsss.h"


typedef struct {
    void           *pvt;
    IOSCANPVT      ioscanpvt;
} dev_pvt;




void process_pidPv(pid_pvt * p)
{
    dev_pvt *dpvt = (dev_pvt *) p->dpvt;
    if(dpvt && dpvt->ioscanpvt) scanIoRequest(dpvt->ioscanpvt);
}

void process_vPv(v_pvt * p)
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

static long init_record_pid(int64inRecord *prec)
{
    char    port[32], key[32];
    int     edef;
    pid_pvt *pvt;
    dev_pvt *dpvt = (dev_pvt *) malloc(sizeof(dev_pvt));
    memset(dpvt, 0, sizeof(dev_pvt));

    switch(prec->inp.type) {
        case INST_IO:
            sscanf(prec->inp.value.instio.string, "%s %s %d", port, key, &edef);
            pvt = find_pidPv(port, key, edef);
            if(!pvt) goto err;
            dpvt->pvt = (void *) pvt;
            pvt->dpvt = (void *) dpvt;
            break;
        default:
            err:
            recGblRecordError(S_db_badField, (void *) prec,
                              "devBsssPid (init_record) Illegal INP field");
            free(dpvt);
            prec->udf = TRUE;
            prec->dpvt = NULL;
            return S_db_badField;
    }

    prec->udf = FALSE;
    prec->dpvt = (void *) dpvt;

    return 0;
}

static long read_pid(int64inRecord *prec)
{
    pid_pvt *p;
    dev_pvt *dpvt = (dev_pvt *) prec->dpvt;
    if(!dpvt) return 0;

    p = (pid_pvt *) dpvt->pvt;
    prec->time = p->time;
    prec->val  = p->pid;


    return 2;  /* no conversion */
}

static long init_record_v(aiRecord *prec)
{
    char    port[32], key[32];
    int     edef;
    v_pvt   *pvt;
    dev_pvt *dpvt = (dev_pvt *) malloc(sizeof(dev_pvt));
    memset(dpvt, 0, sizeof(dev_pvt));

    switch(prec->inp.type) {
        case INST_IO:
            sscanf(prec->inp.value.instio.string, "%s %s %d", port, key, &edef);
            pvt = find_vPv(port, key, edef);
            if(!pvt) goto err;
            dpvt->pvt = (void *) pvt;
            pvt->dpvt = (void *) dpvt;
            break;
        default:
            err:
            recGblRecordError(S_db_badField, (void *) prec,
                              "devBsssPid (init_record) Illegal INP field");
            free(dpvt);
            prec->udf = TRUE;
            prec->dpvt = NULL;
            return S_db_badField;
    }

    prec->udf = FALSE;
    prec->dpvt = (void *) dpvt;


    return 0;
}

static long read_v(aiRecord *prec)
{
    v_pvt *p;
    dev_pvt *dpvt = (dev_pvt *)prec->dpvt;
    if(!dpvt) return 0;

    p = (v_pvt *) dpvt->pvt;
    prec->time = p->time;
    prec->val  = p->v;

    return 2;   /* no conversion */
}



struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
}devBsssPid={
        5,
        NULL,
        NULL,
        init_record_pid,
        get_ioint_info,
        read_pid
};

struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
        DEVSUPFUN       speical_linconv;
}devBsssV={
        6,
        NULL,
        NULL,
        init_record_v,
        get_ioint_info,
        read_v,
        NULL
};

epicsExportAddress(dset, devBsssPid);
epicsExportAddress(dset, devBsssV);


#include <math.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsPrint.h>
#include <ellLib.h>
#include <iocsh.h>

#include <drvSup.h>
#include <epicsExport.h>

#include <yamlLoader.h>
#include "bldStream.h"

#define  MAX_FREE_LIST 8
#define  MAX_BUFF_SIZE 4096

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *listener_name;
    unsigned        read_size;
    unsigned        read_count;
    unsigned        bld_count;
    unsigned        bsss_count;

    void (*bsss_callback)(void *, void *);
    void *pUsrBsss;
    void (*bld_callback)(void *, void *);
    void *pUsrBld;

    ELLLIST         *free_list;

} pDrvList_t;

typedef struct {
    ELLNODE         node;
    char            buff[MAX_BUFF_SIZE];
} pBuff_t;

static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST*) mallocMustSucceed(sizeof(ELLLIST), "bldStream driver: init_drvList()");
        ellInit(pDrvEllList);
    }

    return;
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

static pDrvList_t *get_drvNode(const char *named_root)
{
    pDrvList_t *p = find_drvByNamedRoot(named_root);

    if(!p) {
        p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bldStream driver: get_drvNode()");
        p->named_root = epicsStrDup(named_root);
        p->listener_name = NULL;
        p->bsss_callback = NULL;
        p->pUsrBsss      = NULL;
        p->bld_callback  = NULL;
        p->pUsrBld       = NULL;
        p->read_count = 0;
        p->bld_count  = 0;
        p->bsss_count = 0;
        p->free_list = (ELLLIST*) mallocMustSucceed(sizeof(ELLLIST), "bldStream drivr: get_drvNode()");
        ellInit(p->free_list);

        for(int i = 0; i < MAX_FREE_LIST; i++) {
            pBuff_t *pn = (pBuff_t *) mallocMustSucceed(sizeof(pBuff_t), "bldStream driver: get_drvNode()");
            ellAdd(p->free_list, &pn->node);   
        }

        ellAdd(pDrvEllList, &p->node);

    }
    
    return p;
}

static void listener(pDrvList_t *p)
{
    Path p_root = (p->named_root && strlen(p->named_root))? cpswGetNamedRoot(p->named_root): cpswGetRoot();
    Stream bld_stream = IStream::create(p_root->findByName(BLDSTREAM_NAME));

    while(true) {
        pBuff_t *np = (pBuff_t *) ellFirst(p->free_list);
        ellDelete(p->free_list, &np->node);
        p->read_size = bld_stream->read((uint8_t*)(np->buff), MAX_BUFF_SIZE, CTimeout());

        p->read_count++;
        ellAdd(p->free_list, &np->node);
    }
}


static void createListener(pDrvList_t *p) {
    char name[80];
    sprintf(name, "bldStrm_%s", p->named_root);
    p->listener_name = epicsStrDup(name);

    epicsThreadCreate(name, epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) listener, (void*) p);

}


int registerBsssCallback(const char *named_root, void (*bsss_callback)(void*, void*), void *pUsrBsss)
{

    pDrvList_t *p = get_drvNode(named_root);
    p->bsss_callback = bsss_callback;
    p->pUsrBsss      = pUsrBsss;

   if(!p->listener_name) createListener(p);


    return 0;
}


int registerBldCallback(const char *named_root, void (*bld_callback)(void*, void*), void *pUsrBld)
{
    pDrvList_t *p = get_drvNode(named_root);
    p->bld_callback = bld_callback;
    p->pUsrBld      = pUsrBld;

   if(!p->listener_name) createListener(p);



    return 0;
}


extern "C" {

/* EPICS driver support for bldStreamDriver */

static int bldStreamDriverReport(int interest);
static int bldStreamDriverInitialize(void);

static struct drvet bldStreamDriver = {
    2,
    (DRVSUPFUN) bldStreamDriverReport,
    (DRVSUPFUN) bldStreamDriverInitialize
};

epicsExportAddress(drvet, bldStreamDriver);

static int bldStreamDriverReport(int interest)
{
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        printf("\tBLD Stream for %s\n", p->named_root);
        printf("\t  read size  : %u\n", p->read_size);
        printf("\t  read count : %u\n", p->read_count);
        printf("\t  bld count  : %u\n", p->bld_count);
        printf("\t  bsss count : %u\n", p->bsss_count);
        printf("\t  bld callback : %p\n", p->bld_callback);
        printf("\t  bld_usr      : %p\n", p->pUsrBld);
        printf("\t  bsss_callback: %p\n", p->bsss_callback);
        printf("\t  bsss_usr     : %p\n", p->pUsrBsss);
        printf("\t  free list    : %p\n", p->free_list);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int bldStreamDriverInitialize(void)
{

   /* Implement EPICS driver initialization here */
    init_drvList();

    if(!pDrvEllList) {
        printf("BLD Stream Driver never been configured\n");
        return 0;
    }

    printf("BLD Stream driver: %d of driver instance(s) has (have) been configured\n", ellCount(pDrvEllList));

    /*
    epicsThreadCreate("bsssDrvPoll", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) bsssMonitorPoll, 0);
    */

    return 0;
}


} /* extern "C" */
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

#define  BLD_PACKET  1
#define  BSSS_PACKET 2
#define  BSAS_PACKET 3

#define  MAX_FREE_LIST 8
#define  MAX_BUFF_SIZE 4096

#define  IDX_SERVICE_MASK     5
#define  BSSS_SERVICE_MASK    0x1ff

#define  BSAS_IDTF_LOC        31
#define  BSAS_IDTF_MASK       (0x1 << BSAS_IDTF_LOC)
#define  BSAS_TBLCNT_LOC      20
#define  BSAS_TBLCNT_MASK     (0xf << BSAS_TBLCNT_LOC)
#define  BSAS_TBLID_LOC       16
#define  BSAS_TBLID_MASK      (0Xf < <BSAS_TBLID_LOC)
#define  BSAS_ROWNUM_LOC      0
#define  BSAS_ROWNUM_MASK     (0xffff<<0BSAS_ROWNUM_LOC)

static ELLLIST *pDrvEllList = NULL;

typedef enum {
    none,
    bld_packet,
    bsss_packet,
    bsas_packet
} packet_type_t;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *listener_name;
    unsigned        read_size;
    unsigned        read_count;
    unsigned        bld_count;
    unsigned        bsss_count;
    unsigned        bsas_count;

    void (*bsss_callback)(void *, void *, unsigned);
    void *pUsrBsss;
    void (*bld_callback)(void *, void *, unsigned);
    void *pUsrBld;
    void (*bsas_callback)(void *, void *, unsigned);
    void *pUsrBsas;

    ELLLIST         *free_list;

    void            *p_last_buff;
    void            *p_last_bsss;
    void            *p_last_bsas;
    void            *p_last_bld;

} pDrvList_t;

typedef struct {
    ELLNODE         node;
    packet_type_t   type;
    unsigned        size;
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
        p->bsas_callback = NULL;
        p->pUsrBsas      = NULL;
        p->read_count = 0;
        p->bld_count  = 0;
        p->bsss_count = 0;
        p->bsas_count = 0;
        p->p_last_buff   = NULL;
        p->p_last_bsss   = NULL;
        p->p_last_bsas   = NULL;
        p->p_last_bld    = NULL;
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
        np->size = p->read_size;
        p->p_last_buff = (void*) np;

        uint32_t  *pu32 = (uint32_t *) np->buff;

        if(pu32[IDX_SERVICE_MASK] & BSAS_IDTF_MASK) {   /* bsas */
            p->bsas_count++;
            p->p_last_bsas = (void *) np;
            np->type = bsas_packet;
            if(p->bsas_callback) (p->bsas_callback)(p->pUsrBsas, (void *) np->buff, np->size);
        }
        else if(pu32[IDX_SERVICE_MASK] & BSSS_SERVICE_MASK) { /* bsss */
            p->bsss_count++;
            p->p_last_bsss = (void *) np;
            np->type = bsss_packet;
            if(p->bsss_callback) (p->bsss_callback)(p->pUsrBsss, (void *) np->buff, np->size);
        }
        else {  /* bld */
            p->bld_count++;
            p->p_last_bld = (void *) np;
            np->type = bld_packet;
            if(p->bld_callback) (p->bld_callback)(p->pUsrBld, (void *) np->buff, np->size);
        }

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


int registerBsssCallback(const char *named_root, void (*bsss_callback)(void*, void*, unsigned), void *pUsrBsss)
{

    pDrvList_t *p = get_drvNode(named_root);
    p->bsss_callback = bsss_callback;
    p->pUsrBsss      = pUsrBsss;

   if(!p->listener_name) createListener(p);


    return 0;
}


int registerBldCallback(const char *named_root, void (*bld_callback)(void*, void*, unsigned), void *pUsrBld)
{
    pDrvList_t *p = get_drvNode(named_root);
    p->bld_callback = bld_callback;
    p->pUsrBld      = pUsrBld;

   if(!p->listener_name) createListener(p);



    return 0;
}

int registerBsasCallback(const char *named_root, void (*bsas_callback)(void *, void*, unsigned), void *pUsrBsas)
{
    pDrvList_t *p = get_drvNode(named_root);
    p->bsas_callback = bsas_callback;
    p->pUsrBsas      = pUsrBsas;

    if(!p->listener_name) createListener(p);

    return 0;
}


static void show_bsss_buffer(void *p, unsigned size)
{
    uint32_t *buff = (uint32_t *) p;
    uint64_t *psv  = (uint64_t *) (buff + (size/4) -2);

    printf("\t\t --------------------------------\n");
    printf("\t\t BSSSS Packet: size(%d)\n", size);
    printf("\t\t --------------------------------\n");
    printf("\t\t timestamp, nsec  : %8x\n", *(buff++));
    printf("\t\t timestamp, sec   : %8x\n", *(buff++));
    printf("\t\t pulse id, lower  : %8x\n", *(buff++));
    printf("\t\t pulse id, upper  : %8x\n", *(buff++));
    printf("\t\t channel mask     : %8x\n", *(buff++));
    printf("\t\t service mask     : %8x\n", *(buff++));
    printf("\t\t severity mask    : %16x\n", *psv); 
}


static void show_bsas_buffer(void *p, unsigned size)
{

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
    payload_t     pl[];    // bsas payloader (arbitrary length)
} packet_t;


    packet_t *pk = (packet_t *) p;

    printf("\t\t --------------------------------\n");
    printf("\t\t BSAS Packet: size(%d)\n", size);
    printf("\t\t --------------------------------\n");
    printf("\t\t timestamp (64bit): %16x\n", pk->hd.timestamp);
    printf("\t\t pulse id  (64bit): %16x\n", pk->hd.pulse_id);
    printf("\t\t channel mask     : %8x\n",  pk->hd.channelMask);
    printf("\t\t row number       : %d\n",   pk->hd.row_number);
    printf("\t\t table_count      : %d\n",   pk->hd.table_count);
    printf("\t\t edef_index       : %d\n",   pk->hd.edef_index);
    printf("\t\t byte pad (0x80)  : %2x\n",  pk->hd.byte_pad);

               printf("\t\t PL CH    CNT EVL ESQ FIX     VAL       SUM       SQUARE      MIN      MAX\n");
               printf("\t\t -------------------------------------------------------------------------\n");
    int j =0;
    for(int i = 0; i < 31; i++) {
        if(pk->hd.channelMask & (uint32_t(0x1) << i)) {
            printf("\t\t %2d %2d %6d   %c   %c   %c %8x %8x %12x %8x %8x\n", j, i, pk->pl[j].sample_count,
                                                                                  pk->pl[j].exception_sum? 'E': 'N',
                                                                                  pk->pl[j].exception_var? 'E': 'N',
                                                                                  pk->pl[j].flag_fixed?    'F': 'N',
                                                                                  pk->pl[j].val,
                                                                                  pk->pl[j].sum,
                                                                                  pk->pl[j].sum_square,
                                                                                  pk->pl[j].min,
                                                                                  pk->pl[j].max);
            j++;
        }
    }

}

static void show_bld_buffer(void *p, unsigned size)
{

    printf("\t\t --------------------------------\n");
    printf("\t\t BLD Packet: size(%d)\n", size);
    printf("\t\t --------------------------------\n");
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
        printf("\t  bsas count : %u\n", p->bsas_count);
        printf("\t  bld callback : %p\n", p->bld_callback);
        printf("\t  bld_usr      : %p\n", p->pUsrBld);
        printf("\t  bsss_callback: %p\n", p->bsss_callback);
        printf("\t  bsss_usr     : %p\n", p->pUsrBsss);
        printf("\t  bsas_callback: %p\n", p->bsas_callback);
        printf("\t  bsas_usr     : %p\n", p->pUsrBsas);
        printf("\t  free list    : %p\n", p->free_list);

        if(interest && p->p_last_buff) {
            pBuff_t *np;

            np = (pBuff_t *) p->p_last_bsss;
            if(np) show_bsss_buffer(np->buff, np->size);

            np = (pBuff_t *) p->p_last_bsas;
            if(np) show_bsas_buffer(np->buff, np->size);

            np = (pBuff_t *) p->p_last_bld;
            if(np) show_bld_buffer(np->buff, np->size);
        }

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

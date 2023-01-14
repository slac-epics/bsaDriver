/** @file bldStream.cpp
 *  @brief Resources to deal with BLD/BSA/BSSS packets.
 *
 *  Initializes structures and register callback functions that are called when
 *  a BLD/BSA/BSSS packet is received. The functions here don't process the
 *  packets. They are transferred as-is to the called back functions.
 *
 *  @todo Finish documentation.
 *  @bug No know bugs.
 */

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
#include <epicsMessageQueue.h>
#include <epicsPrint.h>
#include <ellLib.h>
#include <iocsh.h>

#include <drvSup.h>
#include <epicsExport.h>

#include <epicsTime.h>
#include <epicsString.h>

#include <yamlLoader.h>
#include "bldStream.h"

#define  BLD_PACKET  1
#define  BSSS_PACKET 2
#define  BSAS_PACKET 3

#define  MAX_FREE_LIST 1024
#define  MAX_BUFF_SIZE 2048

// number of worker threads
#define  MAX_BSSSQ            4
#define  MAX_BLDQ             1      /* please, do not change it */
#define  MAX_BSASQ            1      /* please, do not change it */

#define  IDX_SERVICE_MASK     5
#define  BSSS_SERVICE_MASK    0x0fffffff

#define  SERVICE_BITS         28
#define  SERVICE_BSSS         1
#define  SERVICE_BLD          2
#define  SERVICE_BSAS         8

#define  BSAS_IDTF_LOC        31
#define  BSAS_IDTF_MASK       (0x1 << BSAS_IDTF_LOC)
#define  BSAS_TBLCNT_LOC      20
#define  BSAS_TBLCNT_MASK     (0xf << BSAS_TBLCNT_LOC)
#define  BSAS_TBLID_LOC       16
#define  BSAS_TBLID_MASK      (0Xf < <BSAS_TBLID_LOC)
#define  BSAS_ROWNUM_LOC      0
#define  BSAS_ROWNUM_MASK     (0xffff<<0BSAS_ROWNUM_LOC)

#ifdef __rtems__
#ifdef __PPC__
#define MFTB(var) asm volatile("mftb %0":"=r"(var))
#else
#define MFTB(var) (var)=Read_timer()
#endif
#endif

#if defined(i386) || defined(__i386) || defined(__i386__) || defined(_M_IX86) || defined(_X86_)  /*  for 32bits */ \
     || defined(__x86_64) || defined(__x86_64__) || defined(_M_X64)                              /*  for 64bits */
/* __inline__ static unsigned long long int rdtsc(void)
{
        unsigned long long int x;
        __asm__ volatile (".byte 0x0f, 0x31": "=A" (x));
        return x;
} */

__inline__ static uint64_t rdtsc(void)
{
   uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return (((uint64_t)hi) << 32) | ((uint64_t)lo);
}

#define MFTB(var)  ((var)=(uint32_t) rdtsc())
#endif

/**
 *  Linked list of pDrvList_t drivers.
 */
static ELLLIST *pDrvEllList = NULL;
static bool listener_ready  = false;
static uint32_t  ticks_in_sec = 0;

typedef enum {
    none,
    bld_packet,
    bsss_packet,
    bsas_packet
} packet_type_t;

/**
 * Structure to hold data related with a specific named root. When and IOC is
 * accessing more than one ATCA crate, then adittional named roots are needed.
 * The linked list pDrvEllList holds nodes of pDrvList_t per named root.
 */
typedef struct {
    ELLNODE         node; /**< Contain pointers to next and previous elements
                               of the linked list. (EPICS ellLib.h) */
    char            *named_root; /**< String representing the named root. */
    char            *listener_name;

    unsigned        read_size;
    unsigned        read_count;
    unsigned        bld_count;
    unsigned        bsss_count;
    unsigned        bsas_count;
    unsigned        else_count;

    struct {
        char    *name;
        void     *q;
        unsigned pend_cnt;
        unsigned water_mark;
        unsigned overrun;
        unsigned fail;
    } bsssQ[MAX_BSSSQ], bldQ[MAX_BLDQ], bsasQ[MAX_BSASQ];


    epicsMessageQueueId   bsasQueue[MAX_BSASQ];
    void (*bsas_callback)(void *, void *, unsigned);
    void *pUsrBsas;

    /**
     * @name Calback functions
     *
     * @{
     */
    epicsMessageQueueId   bsssQueue[MAX_BSSSQ];
    void (*bsss_callback)(void *, 
                          void *, 
                          unsigned); /**< Pointer to function that will be called
                                         when a BSSS packet arrives from the
                                         firmware. */
    void *pUsrBsss; /**< General data that will be available to the function
                        called back when a BSSS packet arrives. Data can be
                        of any type. */
    epicsMessageQueueId  bldQueue[MAX_BLDQ];
    void (*bld_callback)(void *, 
                         void *, 
                         unsigned); /**< Pointer to function that will be called
                                         when a BLD packet arrives from the
                                         firmware. */
    void *pUsrBld; /**< General data that will be available to the function
                        called back when a BLD packet arrives. Data can be
                        of any type. */
    /**@}*/

    ELLLIST         *free_list;

    void            *p_last_buff;
    void            *p_last_bsss0;
    void            *p_last_bsss1;
    void            *p_last_bsas;
    void            *p_last_bld;

} pDrvList_t;

typedef struct {
    ELLNODE         node;
    packet_type_t   type;
    unsigned        size;
    pDrvList_t      *p;
    char            buff[MAX_BUFF_SIZE];
} pBuff_t;

typedef struct {
    int             qid;
    pDrvList_t      *p;
} pUsrPvtQ_t;

/** @brief Initializes pDrvEllList if needed.
 *
 *  Only one instance of pDrvEllList should exist. This function initializes it
 *  only if necessary.
 */
static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST*) mallocMustSucceed(sizeof(ELLLIST), "bldStream driver: init_drvList()");
        ellInit(pDrvEllList);
    }

    return;
}

/** @brief Search among all elements of pDrvEllList for a match in named_root.
 *
 *  @param[in] named_root Named root used when loading the YAML files.
 *  @return A pointer to the driver node associated with named_root.
 */
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

/** @brief Retrieve element of a linked list of drivers, searching for
 *         named_root
 *
 *  If find_dvrByNamedRoot can't find a correspondence with named_root, create
 *  a new pDrvList_t element, initialize it, add to the list of drivers and
 *  return it.
 *
 *  This function will always return a node with a driver, either a new created
 *  one or one that already exists.
 *
 *  @param[in] named_root Named root used when loading the YAML files.
 *  @return A pointer to the driver node associated with named_root.
 */
static pDrvList_t *get_drvNode(const char *named_root)
{
    pDrvList_t *p = find_drvByNamedRoot(named_root);

    if(!p) {
        p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "bldStream driver: get_drvNode()");
        p->named_root = epicsStrDup(named_root);
        p->listener_name = NULL;
        for(int i = 0; i < MAX_BSSSQ; i++) { p->bsssQ[i] = { NULL, 0, 0, 0, 0}; }
        for(int i = 0; i < MAX_BLDQ;  i++) { p->bldQ[i]  = { NULL, 0, 0, 0, 0}; }
        for(int i = 0; i < MAX_BSASQ; i++) { p->bsasQ[i] = { NULL, 0, 0, 0, 0}; }
        for(int i = 0; i < MAX_BSSSQ; i++) { p->bsssQueue[i]     = 0; }
        p->bsss_callback = NULL;
        p->pUsrBsss      = NULL;
        for(int i = 0; i < MAX_BLDQ;  i++) { p->bldQueue[i]      = 0; }
        p->bld_callback  = NULL;
        p->pUsrBld       = NULL;
        for(int i = 0; i < MAX_BSASQ; i++) { p->bsasQueue[i]     = 0; }
        p->bsas_callback = NULL;
        p->pUsrBsas      = NULL;
        p->read_count = 0;
        p->bld_count  = 0;
        p->bsss_count = 0;
        p->bsas_count = 0;
        p->else_count = 0;


        p->p_last_buff   = NULL;
        p->p_last_bsss0  = NULL;
        p->p_last_bsss1  = NULL;
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
        np->p    = p;
        p->p_last_buff = (void*) np;

        uint32_t  *pu32 = (uint32_t *) np->buff;
        int qid;

        if(listener_ready) {
        if(pu32[IDX_SERVICE_MASK]>>SERVICE_BITS == SERVICE_BSAS)  {   /* bsas */
            qid = (p->bsas_count++) % MAX_BSASQ;
            p->p_last_bsas = (void *) np;
            np->type = bsas_packet;
            if(p->bsasQueue[qid]) {
                epicsMessageQueueSend(p->bsasQueue[qid], (void *) &np, sizeof(np));
                p->bsasQ[qid].pend_cnt = epicsMessageQueuePending(p->bsasQueue[qid]);
                if(p->bsasQ[qid].pend_cnt > p->bsasQ[qid].water_mark) p->bsasQ[qid].water_mark = p->bsasQ[qid].pend_cnt;
            }
        }
        else if(pu32[IDX_SERVICE_MASK]>>SERVICE_BITS <= SERVICE_BSSS) { /* bsss, 0: Bsss0, 1: Bsss1 */
            qid = (p->bsss_count++) % MAX_BSSSQ;
            if(pu32[IDX_SERVICE_MASK]>>SERVICE_BITS) p->p_last_bsss1 = (void *) np;
            else                                     p->p_last_bsss0 = (void *) np;
            np->type = bsss_packet;
            if(p->bsssQueue[qid]) {
                epicsMessageQueueSend(p->bsssQueue[qid], (void *) &np, sizeof(np));
                p->bsssQ[qid].pend_cnt = epicsMessageQueuePending(p->bsssQueue[qid]);
                if(p->bsssQ[qid].pend_cnt > p->bsssQ[qid].water_mark) p->bsssQ[qid].water_mark = p->bsssQ[qid].pend_cnt;
            }
        }
        else if(pu32[IDX_SERVICE_MASK]>>SERVICE_BITS == SERVICE_BLD) { /* bld */
            qid = (p->bld_count++) % MAX_BLDQ;
            p->p_last_bld = (void *) np;
            np->type = bld_packet;
            if(p->bldQueue[qid]) {
                epicsMessageQueueSend(p->bldQueue[qid], (void *) &np, sizeof(np));
                p->bldQ[qid].pend_cnt = epicsMessageQueuePending(p->bldQueue[qid]);
                if(p->bldQ[qid].pend_cnt > p->bldQ[qid].water_mark) p->bldQ[qid].water_mark = p->bldQ[qid].pend_cnt;
            }
        } else p->else_count++;  // something wrong, packet could not be specified
        }  // if(listener_ready)

        p->read_count++;
        ellAdd(p->free_list, &np->node);
    }
}

static void bsssQTask(void *usrPvt)
{
    pUsrPvtQ_t  *q = (pUsrPvtQ_t *) usrPvt;
    pDrvList_t  *p = q->p;
    int        qid = q->qid;
    pBuff_t   *np;

    while(true) {
        int msg = epicsMessageQueueReceive(p->bsssQueue[qid], (void *) &np, sizeof(np));
        if(msg != sizeof(np)) {p->bsssQ[qid].fail++; continue;}

        if(np->type == bsss_packet) {
            (p->bsss_callback)(p->pUsrBsss, (void *) np->buff, np->size);
        } else p->bsssQ[qid].overrun++;
    }
}

static void bldQTask(void *usrPvt)
{
    pUsrPvtQ_t  *q = (pUsrPvtQ_t *) usrPvt;
    pDrvList_t  *p = q->p;
    int        qid = q->qid;
    pBuff_t   *np;

    while(true) {
        int msg = epicsMessageQueueReceive(p->bldQueue[qid], (void *) &np, sizeof(np));
        if(msg != sizeof(np)) {p->bldQ[qid].fail++; continue;}

        if(np->type == bld_packet) {
            (p->bld_callback)(p->pUsrBld, (void *) np->buff, np->size);
        } else p->bldQ[qid].overrun++;
    }
}

static void bsasQTask(void *usrPvt)
{
    pUsrPvtQ_t  *q = (pUsrPvtQ_t *) usrPvt;
    pDrvList_t  *p = q->p;
    int        qid = q->qid;
    pBuff_t   *np;

    while(true) {
        int msg = epicsMessageQueueReceive(p->bsasQueue[qid], (void *) &np, sizeof(np));
        if(msg != sizeof(np)) {p->bsasQ[qid].fail++; continue;}

        if(np->type == bsas_packet) {
            (p->bsas_callback)(p->pUsrBsas, (void *) np->buff, np->size);
        } else p->bsasQ[qid].overrun++;
    }
}

static void createListener(pDrvList_t *p) {
    char name[80];
    sprintf(name, "bldStrm_%s", p->named_root);
    p->listener_name = epicsStrDup(name);

    epicsThreadCreate(name, epicsThreadPriorityHigh-10,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) listener, (void*) p);

}

static void createBsssQTask(pUsrPvtQ_t *q) {
    int       qid = q->qid;
    pDrvList_t *p = q->p;
    char name[80];
    sprintf(name, "bsssQ%d_%s", qid, p->named_root);
    p->bsssQ[qid].name = epicsStrDup(name);
    p->bsssQueue[qid] = epicsMessageQueueCreate(MAX_FREE_LIST, sizeof(pBuff_t *));

    epicsThreadCreate(name, epicsThreadPriorityMedium+15,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) bsssQTask, (void *) q); 
}

static void createBldQTask(pUsrPvtQ_t *q) {
    int       qid = q->qid;
    pDrvList_t *p = q->p;
    char name[80];
    sprintf(name, "bldQ%d_%s", qid, p->named_root);
    p->bldQ[qid].name = epicsStrDup(name);
    p->bldQueue[qid] = epicsMessageQueueCreate(MAX_FREE_LIST, sizeof(pBuff_t *));

    epicsThreadCreate(name, epicsThreadPriorityMedium+15,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) bldQTask, (void *) q);
}

static void createBsasQTask(pUsrPvtQ_t *q) {
    int       qid = q->qid;
    pDrvList_t *p = q->p;
    char name[80];
    sprintf(name, "bsasQ%d_%s", qid, p->named_root);
    p->bsasQ[qid].name = epicsStrDup(name);
    p->bsasQueue[qid] = epicsMessageQueueCreate(MAX_FREE_LIST , sizeof(pBuff_t *));

    epicsThreadCreate(name, epicsThreadPriorityMedium+15,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) bsasQTask, (void *) q);
}


int registerBsssCallback(const char *named_root, void (*bsss_callback)(void*, void*, unsigned), void *pUsrBsss)
{

    pDrvList_t *p = get_drvNode(named_root);
    p->bsss_callback = bsss_callback;
    p->pUsrBsss      = pUsrBsss;

   if(!p->listener_name) createListener(p);

   for(int i = 0; i < MAX_BSSSQ; i++) {
       if(!p->bsssQueue[i]) {
           pUsrPvtQ_t *q = (pUsrPvtQ_t *) mallocMustSucceed(sizeof(pUsrPvtQ_t), "registerBsssCallback");
           q->qid = i;
           q->p   = p;
           p->bsssQ[i].q = q;
           
           createBsssQTask(q);
        }
   }


    return 0;
}


int registerBldCallback(const char *named_root, void (*bld_callback)(void*, void*, unsigned), void *pUsrBld)
{
    pDrvList_t *p = get_drvNode(named_root);
    p->bld_callback = bld_callback;
    p->pUsrBld      = pUsrBld;

   if(!p->listener_name) createListener(p);

    for(int i = 0; i < MAX_BLDQ; i++) {
        if(!p->bldQueue[i]) {
           pUsrPvtQ_t *q = (pUsrPvtQ_t *) mallocMustSucceed(sizeof(pUsrPvtQ_t), "registerBldCallback");
           q->qid = i;
           q->p   = p;
           p->bldQ[i].q = q;

           createBldQTask(q);
        }
    }

    return 0;
}

int registerBsasCallback(const char *named_root, void (*bsas_callback)(void *, void*, unsigned), void *pUsrBsas)
{
    pDrvList_t *p = get_drvNode(named_root);
    p->bsas_callback = bsas_callback;
    p->pUsrBsas      = pUsrBsas;

    if(!p->listener_name) createListener(p);

    for (int i = 0; i < MAX_BSASQ; i++) {
        if(!p->bsasQueue[i]) {
           pUsrPvtQ_t *q = (pUsrPvtQ_t *) mallocMustSucceed(sizeof(pUsrPvtQ_t), "registerBsasCallback");
           q->qid = i;
           q->p   = p;
           p->bsasQ[i].q = q;

           createBsasQTask(q);
        }
    }
    return 0;
}


static void show_bsss_buffer(void *p, unsigned size)
{
    uint32_t *buff = (uint32_t *) p;
    uint64_t *psv  = (uint64_t *) (buff + (size/4) -2);
    int n;

    if (!((buff[IDX_SERVICE_MASK] >> SERVICE_BITS) <= SERVICE_BSSS) )
    {
        printf("\t\t --------------------------------\n");
        printf("\t\t BSSS last packet flushed\n");
        printf("\t\t --------------------------------\n");   
        return;
    }   

    if(buff[IDX_SERVICE_MASK]>>SERVICE_BITS) n = 1;
    else                                     n = 0;

    printf("\t\t --------------------------------\n");
    printf("\t\t BSSS%d Packet: size(%d)\n", n, size);
    printf("\t\t --------------------------------\n");
    printf("\t\t timestamp, nsec  : %8x\n", *(buff++));
    printf("\t\t timestamp, sec   : %8x\n", *(buff++));
    printf("\t\t pulse id, lower  : %8x\n", *(buff++));
    printf("\t\t pulse id, upper  : %8x\n", *(buff++));
    printf("\t\t channel mask     : %8x\n", *(buff++));
    printf("\t\t service mask     : %8x\n", *(buff++));
    printf("\t\t severity mask    : %16llx\n", (long long unsigned int)(*psv)); 
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
    uint8_t  byte_pad:      4;
    uint8_t  serviceMask:   4;
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

    if(pk->hd.serviceMask != SERVICE_BSAS) {
        printf("\t\t --------------------------------\n");
        printf("\t\t BSAS last packet flushed\n");
        printf("\t\t --------------------------------\n");
        return;
    }

    printf("\t\t --------------------------------\n");
    printf("\t\t BSAS Packet: size(%d)\n", size);
    printf("\t\t --------------------------------\n");
    printf("\t\t timestamp (64bit)  : %16llx\n", (long long unsigned int)(pk->hd.timestamp));
    printf("\t\t pulse id  (64bit)  : %16llx\n", (long long unsigned int)(pk->hd.pulse_id));
    printf("\t\t channel mask       : %8x\n",  pk->hd.channelMask);
    printf("\t\t row number         : %d\n",   pk->hd.row_number);
    printf("\t\t table_count        : %d\n",   pk->hd.table_count);
    printf("\t\t edef_index         : %d\n",   pk->hd.edef_index);
    printf("\t\t byte pad           : %2x\n",  pk->hd.byte_pad);
    printf("\t\t service mask (0x80): %2x\n", pk->hd.serviceMask);

               printf("\t\t PL CH    CNT EVL ESQ FIX     VAL       SUM       SQUARE      MIN      MAX\n");
               printf("\t\t -------------------------------------------------------------------------\n");
    int j =0;
    for(int i = 0; i < 31; i++) {
        if(pk->hd.channelMask & (uint32_t(0x1) << i)) {
            printf("\t\t %2d %2d %6d   %c   %c   %c %8x %8x %12llx %8x %8x\n", j, i, pk->pl[j].sample_count,
                                                                                  pk->pl[j].exception_sum? 'E': 'N',
                                                                                  pk->pl[j].exception_var? 'E': 'N',
                                                                                  pk->pl[j].flag_fixed?    'F': 'N',
                                                                                  pk->pl[j].val,
                                                                                  pk->pl[j].sum,
                                                                                  (long long unsigned int) (pk->pl[j].sum_square),
                                                                                  pk->pl[j].min,
                                                                                  pk->pl[j].max);
            j++;
        }
    }

}

static void show_bld_buffer(void *p, unsigned size)
{

    typedef struct __attribute__((__packed__)) {
        uint64_t timeStamp;
        uint64_t pulseID;
        uint32_t channelMask;
        uint32_t serviceMask;
    } bldAxiStreamHeader_t;

    typedef struct __attribute__((__packed__)) {
        uint32_t deltaTimeStamp:20;
        uint32_t deltaPulseID:12;
        uint32_t serviceMask;
    } bldAxiStreamComplementaryHeader_t;

    uint32_t *buff = (uint32_t *) p;
    bldAxiStreamHeader_t *header = (bldAxiStreamHeader_t *) p;
    bldAxiStreamComplementaryHeader_t * compHeader;
    uint64_t severityMask;
    uint32_t consumedWords = 0;
    int channelsFound = 0;
    const uint8_t wordSize = 4;

    if ((header->serviceMask >> SERVICE_BITS) != SERVICE_BLD )
    {
        printf("\t\t --------------------------------\n");
        printf("\t\t BLD last packet flushed\n");
        printf("\t\t --------------------------------\n");   
        return;
    }   

    printf("\t\t --------------------------------\n");
    printf("\t\t BLD Packet: size(%d)\n", size);
    printf("\t\t --------------------------------\n");
    printf("\t\t timestamp        : %16lx\n", header->timeStamp);
    printf("\t\t pulse ID         : %16lx\n", header->pulseID);
    printf("\t\t channel mask     :         %8x\n", header->channelMask);
    printf("\t\t service mask     :         %8x\n", header->serviceMask);    



    consumedWords += sizeof(bldAxiStreamHeader_t)/4;
    
    for (uint32_t channel_mask_it=0x1 ; channel_mask_it != 0x0; channel_mask_it<<= 1)
    {
        if (header->channelMask & channel_mask_it)
            channelsFound++;
    }

    printf("\t\t Chan. data found :               %d\n", channelsFound); 
    consumedWords += channelsFound;
    
    severityMask =  *((uint64_t *) (buff + consumedWords));
    printf("\t\t Severity mask    : %16lx\n", severityMask);

    consumedWords += sizeof(severityMask)/wordSize;
    const uint32_t amendedEventSize = sizeof(bldAxiStreamComplementaryHeader_t) + channelsFound*wordSize + sizeof(severityMask);

    while (consumedWords*wordSize < size)
    {
        if (consumedWords*wordSize + amendedEventSize > size){
            printf("\t\t Anomaly detected: Last event data corrupt\n");
            return;
        }
        else
            printf("\t\t Amended event data\n");

        compHeader = (bldAxiStreamComplementaryHeader_t *) (buff + consumedWords);
        
        printf("\t\t\t Delta timestamp  : %8x\n", compHeader->deltaTimeStamp);
        printf("\t\t\t Delta Pulse ID   : %8x\n", compHeader->deltaPulseID);
        printf("\t\t\t Service mask     : %8x\n", compHeader->serviceMask);

        consumedWords += sizeof(bldAxiStreamComplementaryHeader_t)/wordSize + channelsFound;

        severityMask =  *((uint64_t *) (buff + consumedWords));
        printf("\t\t\t Severity mask    : %16lx\n", severityMask);

        consumedWords += sizeof(severityMask)/wordSize;
    } 
        
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
        printf("\t  else count : %u\n", p->else_count);
        printf("\t  bld callback : %p\n", p->bld_callback);
        printf("\t  bld_usr      : %p\n", p->pUsrBld);
        printf("\t  bsss_callback: %p\n", p->bsss_callback);
        printf("\t  bsss_usr     : %p\n", p->pUsrBsss);
        printf("\t  bsas_callback: %p\n", p->bsas_callback);
        printf("\t  bsas_usr     : %p\n", p->pUsrBsas);
        printf("\t  free list    : %p\n", p->free_list);

        for(int i = 0; i < MAX_BLDQ; i++) {
            printf("\t  bldQueue[%d]\n", i);
            printf("\t\t name of thread: %s\n", p->bldQ[i].name);
            printf("\t\t pending count : %u\n", p->bldQ[i].pend_cnt);
            printf("\t\t water mark    : %u\n", p->bldQ[i].water_mark);
            printf("\t\t fail  count   : %u\n", p->bldQ[i].fail);
            printf("\t\t overrun count : %u\n", p->bldQ[i].overrun);
        }

        for(int i = 0; i < MAX_BSSSQ; i++) {  
            printf("\t  bsssQueue[%d]\n", i);
            printf("\t\t name of thread: %s\n", p->bsssQ[i].name);
            printf("\t\t pending count : %u\n", p->bsssQ[i].pend_cnt);
            printf("\t\t water mark    : %u\n", p->bsssQ[i].water_mark);
            printf("\t\t fail  count   : %u\n", p->bsssQ[i].fail);
            printf("\t\t overrun count : %u\n", p->bsssQ[i].overrun);
        }

        for (int i = 0; i < MAX_BSASQ; i++) { 
            printf("\t  bsasQueue[%d]\n", i);
            printf("\t\t name of thread: %s\n", p->bsasQ[i].name);
            printf("\t\t pending count : %u\n", p->bsasQ[i].pend_cnt);
            printf("\t\t water mark    : %u\n", p->bsasQ[i].water_mark);
            printf("\t\t fail  count   : %u\n", p->bsasQ[i].fail);
            printf("\t\t overrun count : %u\n", p->bsasQ[i].overrun);
        }

        if(interest > 10) {  // reset min and max for the processing time mesasurement
            for(int i = 0; i < MAX_BLDQ; i++) {
                p->bldQ[i].water_mark = 0;
                p->bldQ[i].fail       = 0;
                p->bldQ[i].overrun    = 0;
            }

            for(int i = 0; i < MAX_BSSSQ; i++) {
                p->bsssQ[i].water_mark = 0;
                p->bsssQ[i].fail       = 0;
                p->bsssQ[i].overrun    = 0;
            }

            for(int i = 0; i < MAX_BSASQ; i++) {
                p->bsasQ[i].water_mark = 0;
                p->bsasQ[i].fail       = 0;
                p->bsasQ[i].overrun    = 0;
            }

            p->read_count  = 0;
            p->bld_count   = 0;
            p->bsss_count  = 0;
            p->bsas_count  = 0;
            p->else_count  = 0;
        }

        if(interest && p->p_last_buff) {
            pBuff_t *np;

            np = (pBuff_t *) p->p_last_bsss0;
            if(np) show_bsss_buffer(np->buff, np->size);   // report for bsss0 packet

            np = (pBuff_t *) p->p_last_bsss1;
            if(np) show_bsss_buffer(np->buff, np->size);   // report for bsss1 packet

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

    uint32_t tick_start, tick_stop;
    do {
        MFTB(tick_start); epicsThreadSleep(1.); MFTB(tick_stop);
    } while(tick_start >= tick_stop);

    ticks_in_sec = tick_stop - tick_start;


    listener_ready = true;     // postpone to activate listener until the driver instialization is done

    return 0;
}


} /* extern "C" */

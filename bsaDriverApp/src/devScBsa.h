#ifndef DEV_SCBSA_H
#define DEV_SCBSA_H


#define PARAM_BSAMEAN "BSAMEAN"
#define PARAM_BSANUM  "BSANUM"
#define PARAM_BSARMS2 "BSARMS2"
#define PARAM_BSAPID  "BSAPID"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *dpvt;
    epicsTimeStamp ts;
    int  nreq;
    int  entry_sz;
    void *bptr;
} devBsaPvt_t;


devBsaPvt_t * find_bsa_pvt(char *port, char *key, char *param, int edef);
void process_bsa_pv(devBsaPvt_t *p);


#ifdef __cplusplus
}      /* extern C */
#endif


#endif /* DEV_SCBSA_H */

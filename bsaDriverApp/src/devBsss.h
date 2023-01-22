#ifndef DEV_BSSS_H
#define DEV_BSSS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void          *dpvt;
    uint64_t       pid;
    epicsTimeStamp time;
} pid_pvt;


typedef struct {
    void          *dpvt;
    double         v;
    epicsTimeStamp time;
} v_pvt;


pid_pvt * find_pidPv(char *port, char *key, int edef);
v_pvt   * find_vPv(char *port, char *key, int edef);
void process_pidPv(pid_pvt *);
void process_vPv(v_pvt *);


#ifdef __cplusplus
}
#endif

#endif

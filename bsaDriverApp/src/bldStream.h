#ifndef BLD_STREAM_H
#define BLD_STREAM_H



#include <cpsw_api_user.h>

#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>

#define BLDSTREAM_NAME "bldstream"


int registerBsssCallback(const char *named_root, void (*bsss_callback)(void *pUsr, void *buff, unsigned size), void *pUsr);
int registerBldCallback(const char *named_root, void (*bld_callback)(void *pUsr, void *buff, unsigned size), void *pUsr);
int registerBsasCallback(const char *named_root, void(*bsas_callback)(void *pUsr, void *buff, unsigned isze), void *PUsr);


#endif /* BLD_STREAM_H */


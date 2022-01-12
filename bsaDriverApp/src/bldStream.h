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


int registerBsssCallback(const char *named_root, void (*bsss_callback)(void *pUsr, void *buff), void *pUsr);
int registerBldCallback(const char *named_root, void (*bld_callback)(void *pUsr, void *buff), void *pUsr);


#endif /* BLD_STREAM_H */


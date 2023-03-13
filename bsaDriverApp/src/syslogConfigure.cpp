#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

#include <syslog.h>

#include <iocsh.h>
#include <epicsExport.h>


extern "C" {


void syslogConfigure(char *arg0, char *arg1)
{
    static int init = 0;

    if(!strcmp(arg1, "upto_debug")) {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    } else if(!strcmp(arg1, "debug")) {
       setlogmask(LOG_DEBUG);
    } else if(!strcmp(arg1, "stop") && init) {
        init = 0;
        closelog();
    } else { 
      setlogmask(0);
    }
    if(!init) {
        init = 1;
        openlog(arg0, LOG_PERROR | LOG_PID | LOG_NDELAY, LOG_USER);
    }



}

static const iocshArg     syslogArg0 = {"log id (string id)", iocshArgString};
static const iocshArg     syslogArg1 = {"max log priority (ex, upto_debug, debug)", iocshArgString};
static const iocshArg * const syslogArgs[] = { &syslogArg0, 
                                               &syslogArg1 };
static const iocshFuncDef syslogFuncDef = {"syslogConfigure", 2, syslogArgs};

static void syslogCallFunc(const iocshArgBuf *args)
{
    syslogConfigure(args[0].sval, args[1].sval);
}

void syslogRegister(void) {
    iocshRegister(&syslogFuncDef,  syslogCallFunc);
}

epicsExportRegistrar(syslogRegister);


}  // extern C



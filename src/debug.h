#ifndef SRC_DEBUG_H_
#define SRC_DEBUG_H_

#include "status_codes.h"
#include "time.h"
#include <stdarg.h>
#include <string.h>
#include "usart_support.h"

#define MAX_STRING_LENGTH 256
//#define DEBUG_LOG_ENABLE

enum {
    MODUL_DEFAULT = 1, MODUL_I2C = 2, MODUL_TOF = 4,MODUL_ALL = 0x7fffffff //all bits except sign
};

#ifdef DEBUG_LOG_ENABLE
#define DEBUG_GET_TIME() getTimeMs()

#define DEBUG_WAIT(modul,fmt,... ) \
        debugUart(modul,"%lu "fmt"\n",DEBUG_GET_TIME(),##__VA_ARGS__);

void debugUart(uint32_t module, const char *format, ...);

#else

#define DEBUG_WAIT(fmt, ... ) (void)0

#endif

#endif /* SRC_DEBUG_H_ */

#ifndef _TIME_H_
#define _TIME_H_

#include <stdint.h>
#include "tc_support.h"

// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t ;

static inline timeDelta_t cmpTimeMs(timeMs_t a, timeMs_t b) { return (timeDelta_t)(a - b); }
static inline timeMs_t getTimeMs(void) { return tc1_ticks; }


#endif /* _TIME_H_ */

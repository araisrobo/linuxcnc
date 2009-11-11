#ifndef _DPTRACE_H_
#define _DPTRACE_H_

#include <stdio.h>

#ifndef TRACE
#define TRACE   1      //!< 0:Trace off 1:Trace on
#endif

#if (TRACE)
#define DP(fmt, args...)                                                \
    do {                                                                \
        fprintf(dptrace, "%s: (%s:%d) ",                                \
                         __FILE__, __FUNCTION__, __LINE__ );            \
        fprintf(dptrace, fmt, ##args);                                  \
        fflush(dptrace);                                                \
    } while (0)
#define DPS(fmt, args...)                                               \
    do {                                                                \
        fprintf(dptrace, fmt, ##args);                                  \
        fflush(dptrace);                                                \
    } while (0)
#else 
// TRACE == 0
#define DP(fmt, args...)     do {} while(0)
#define DPS(fmt, args...)    do {} while(0)
#endif

#endif  // _DPTRACE_H_

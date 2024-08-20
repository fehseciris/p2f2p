#include "oswrappers.h"

/* Work with time */
void
oswrapper::localtime(const time_t* time_time_t, tm* time_tm)
{
#ifdef __linux__
    localtime_r(time_time_t, time_tm);
#else
    localtime_s(time_tm, time_time_t);
#endif
}

/* Eof */
#include "Utils.h"

//#include <fstream>
//#include <iostream>
#include <cassert>
#include <memory>

#include <mach/mach.h>
#include <mach/mach_time.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
uint64_t GetProcessTime()
{
    rusage ru;
    if( getrusage( RUSAGE_SELF, &ru ) != -1 )
    {
        const uint64_t ms = ru.ru_utime.tv_sec * 1000 + ru.ru_utime.tv_usec / 1000;
        return ms;
    }
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
uint64_t GetRealTimeNano()
{
    static mach_timebase_info_data_t sTimebaseInfo;
    
    if( 0 == sTimebaseInfo.denom )
        mach_timebase_info( &sTimebaseInfo );
        
    const uint64_t thisTime = mach_absolute_time();
    const uint64_t timeNano = thisTime * sTimebaseInfo.numer / sTimebaseInfo.denom;
    return timeNano;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

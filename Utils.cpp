#include "Utils.h"

//#include <fstream>
//#include <iostream>
#include <cassert>
#include <memory>

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

#ifndef TIMING_H
#define TIMING_H

#include <sys/time.h>
#include <iostream>
#include <chrono>

namespace timing {
    /** @brief start counting, the value is not timestamp.*/
    #define TIME_COUNT (std::chrono::duration_cast<std::chrono::duration<double>>(\
                        std::chrono::steady_clock::now().time_since_epoch())).count()

    using namespace std;    
}

namespace timing {

/** @brief Get time stamp. */
inline
double timeStamp()
{
    struct timeval tv; 
    gettimeofday(&tv, NULL);
    return (tv.tv_sec + tv.tv_usec / 1000000.0);
}

/** @brief Get time difference from last time stamp. */
inline
double timeStampDiff(double & time_stamp_last)
{
    struct timeval tv; 
    gettimeofday(&tv, NULL);
    double time_stamp_now = tv.tv_sec + tv.tv_usec / 1000000.0;
    return (time_stamp_now - time_stamp_last);
}

}

#endif


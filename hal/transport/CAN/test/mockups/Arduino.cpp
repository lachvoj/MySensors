
#include <sys/time.h>
#include <time.h>

static unsigned long millis_at_start = 0;

unsigned long micros()
{
    timeval curTime;

    if (millis_at_start == 0)
    {
        gettimeofday(&curTime, NULL);
        millis_at_start = curTime.tv_sec;
    }

    gettimeofday(&curTime, NULL);
    return ((curTime.tv_sec - millis_at_start) * 1000000) + (curTime.tv_usec);
}
#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>

namespace time_utils
{
    /** Various wallclock time utilities. May need to be updated for Windows / OSX */

    typedef struct timeval timestamp;
    
    static inline
	timestamp now(void)
    {	
	timestamp start;
	gettimeofday(&start, NULL);
	return start;
    }
    
    static inline
	void now(timestamp &start)
    {	
	gettimeofday(&start, NULL);
    }
    
    static inline
	double elapsed(timestamp &start)
    {
	timestamp end;
	gettimeofday(&end, NULL);
	return (end.tv_sec - start.tv_sec) + 
	    0.000001 * (end.tv_usec - start.tv_usec);
    }
    
    static inline
	bool sleep_ns(long nsec)
    {
	struct timespec ts = {0, nsec};
	return nanosleep(&ts, NULL) == 0;
    }
    
    static inline
	bool sleep_us(long usec)
    {
	struct timespec ts = {usec / 1000000, 1000 * (usec % 1000000)};
	return nanosleep(&ts, NULL) == 0;
    }
    
    static inline
	bool sleep_ms(unsigned int msec)
    {
	struct timespec ts = {msec / 1000, 1000000 * (msec % 1000)};
	return nanosleep(&ts, NULL) == 0;
    }
    
    static inline
	bool sleep_s(unsigned int sec)
    {
	struct timespec ts = {sec, 0};
	return nanosleep(&ts, NULL) == 0;
    }
    
    static inline
	const char* timestr(void)
    {
	time_t rawtime;
	time(&rawtime);
	return ctime(&rawtime);
    }
    
}

#endif


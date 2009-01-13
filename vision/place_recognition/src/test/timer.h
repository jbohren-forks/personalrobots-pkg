#ifndef TEST_TIMER_H
#define TEST_TIMER_H

#include <sys/resource.h>

class Timer
{
public:
  Timer() {
    restart();
  }

  void restart() {
    getrusage(RUSAGE_SELF, &m_start);
  }
  
  double elapsed() const {
    rusage end;
    getrusage(RUSAGE_SELF, &end);
    double start_time = m_start.ru_utime.tv_sec + m_start.ru_utime.tv_usec/1000000.0;
    double end_time = end.ru_utime.tv_sec + end.ru_utime.tv_usec/1000000.0;
    double seconds = end_time - start_time;
    return seconds;
  }

private:
  rusage m_start;
};

#endif

#ifndef TEST_TIMER_H
#define TEST_TIMER_H

//#include <ctime>
#include <sys/time.h>
#include <sys/resource.h>
#include <string>
#include <cstdio>

class Timer
{
public:
  Timer(std::string name = "Timer") : m_name(name)
  {
    //m_start = clock();
    getrusage(RUSAGE_SELF, &m_start);
  }

  ~Timer()
  {
    //clock_t end = clock();
    //float seconds = (float)(end - m_start) / CLOCKS_PER_SEC;
    rusage end;
    getrusage(RUSAGE_SELF, &end);
    double start_time = m_start.ru_utime.tv_sec + m_start.ru_utime.tv_usec/1000000.0;
    double end_time = end.ru_utime.tv_sec + end.ru_utime.tv_usec/1000000.0;
    double seconds = end_time - start_time;
    printf("%s: %f s\n", m_name.c_str(), seconds);
  }

private:
  std::string m_name;
  //clock_t m_start;
  rusage m_start;
};

#endif

#include <iostream>
#include <assert.h>
#include <unistd.h>
#include <stdint.h>

#include "timer.h"

#define FAKE 1


const int Timer::invMicroSecond(1000000);
const double Timer::microSecond(1.0e-6);
const unsigned int Timer::sleepLimit(10000); // based on default 250 HZ kernel timer
const unsigned int Timer::sleepAdjust(1000);

struct timeval Timer::t;

double Timer::scale = 1.0;  //What to scale time by
bool Timer::fakeTime = false; //should we scale time
us_time Timer::timeBase = 0;


us_time Timer::RealTime()
{
  gettimeofday( &t, 0);

  return (((us_time) t.tv_usec) +
      ((us_time) t.tv_sec) * ((us_time) invMicroSecond));
}

us_time Timer::GetTimeUS()
{
  us_time timeResult;

#if (FAKE == 1)
  if(fakeTime)
  {
    timeResult = (us_time)((double)(RealTime() - timeBase) * scale) + timeBase;
  }
  else
  {
    timeResult = RealTime();
  }
#else
    timeResult = RealTime();
#endif

  return timeResult;
}

float Timer::GetTimeSeconds()
{
  float timeResult;

#if (FAKE == 1)
  if(fakeTime)
  {
    timeResult = ((us_time)((double)(RealTime() - timeBase) * scale) + timeBase) / 1000000.0;
  }
  else
  {
    timeResult = RealTime() / 1000000.0;
  }
#else
    timeResult = RealTime() / 1000000.0;
#endif

  return timeResult;
}

Timer::Timer( us_time usDelay )
{
  Delay( usDelay );
}

void Timer::Delay( us_time usDelay )
{
  assert(usDelay >= 0);

  requestedDelay = usDelay;
  Start();
}

void Timer::Start()
{
  finishTime = GetTimeUS() + requestedDelay;
}

bool Timer::Done()
{
  if( GetTimeUS() > ( finishTime ) )
    return true;

  return false;
}

long int Timer::Remaining()
{
  if(Done())
    return 0;

  long int leftover = finishTime - GetTimeUS();
  return leftover;
}

#if 0
void Timer::strf( char *string, int max_size, const char *format, us_time when)
{
  struct tm tm;
  time_t timet;

  timet = (time_t)(when * microSecond);
  localtime_r(&timet, &tm);

  strftime(string, max_size, format, &tm);
}

char *Timer::strp(const char *s, const char *format, us_time *when)
{
  time_t timet;
  struct tm tm;
  char *ptr;

  time(&timet);       // load daylight savings and tz.
  localtime_r(&timet, &tm);

  ptr=strptime(s, format, &tm);
  if (!ptr)
    return NULL;

  timet=mktime(&tm);
  *when=((timestamp)(timet)) * invMicroSecond;
  return (ptr);
}
#endif

void Timer::Wait()
{
  unsigned int delay = finishTime - GetTimeUS();
  if( !Done() )
  {
    if(delay > sleepLimit)
    {
      if(delay > 10000000)
      {
        std::cerr << "Wow, big delay delay=" << delay << "\n";
        sleep( (unsigned int)(delay * microSecond) );
      }
      else
      {
#if (TARGET == ARM)
        usleep(delay - sleepAdjust);
#else
        if(usleep(delay - sleepAdjust) != 0)
        {
          std::cerr << "autoboat::library::time::wait Error returned from usleep" << std::endl;
        }
#endif
      }
    }

    while( !Done() )
      ;
  }
}

void Timer::SetTimeScale( double newScale)
{
#if (FAKE == 1)
  timeBase = RealTime(); //record the time when we started scaling
  fakeTime = true;
  scale = newScale;
#endif
//  std::cout << "setTimeScale: timeBase=" << timeBase << " fakeTime=" << fakeTime << " scale=" << scale << " \n";
}

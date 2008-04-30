#pragma once
#include <time.h>
#include <iostream>
#include <sys/time.h>


typedef unsigned long int us_time;
class Timer
{
  public:
    /*!  \brief Get the time in microseconds, can be real or fake simulator time.
    */
    static us_time GetTimeUS();
    /*!  \brief Get the time in seconds, can be real or fake simulator time.
    */
    static float GetTimeSeconds();

    /*!  \brief Get a 64 bit count from the processor

      This function returns the 64 bit value of a hardware counter
      implemented in the processor. This should not be used as a timer
      because it is clocked by a source that changes as the processor clock
      is scaled up and down. Also these counters are unique to each processor
      core and may not be synchronized.
      */
      static uint64_t rdtsc() {
      uint32_t lo, hi;
      /* We cannot use "=A", since this would use %rax on x86_64 */
      __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
      return (uint64_t)hi << 32 | lo;
      }

    /*! \brief Create a timer.
      *
      * You use this to measure time or wait for a time delay.
      * This is optomized to sleep for long delays or just burn cycles
      * for short delays, which makes it more accurate. It also works
      * when running with a simulator.
      *
      * \param usDelay initialize the object with this delay in micro-seconds.
      */
    Timer( unsigned long int usDelay = 0 );
    virtual ~Timer() { }

    /*! \brief Set the delay to this new value.
      *
      * \param usDelay change the delay to this new value in micro-seconds.
      */
    void Delay( us_time usDelay );
    /*! \brief Start the timer.
      */
    void Start();
    /*! \brief Return true when the programmed delay has been exceeded.
      */
    bool Done();
    /*! \brief Return the remaing delay in micro-seconds, negative 
      * means the delay has been exceeded.
      */
    long int Remaining(); // if the delay is not done, returns the remaining time
    /*! \brief Return the programed delay value.
      */
    us_time GetDelay() { return requestedDelay; }
    /*! \brief Sit and wait for the remaining time to expire. Returns when the time has passed.
      */
    void Wait();  //wait until delay is finished.
    
    static void SetTimeScale(double newScale);
    static double GetTimeScale() { return scale; };

    static const int invMicroSecond;
    static const double microSecond;
    static const unsigned int sleepLimit;
    static const unsigned int sleepAdjust;

    //static void strf( char *string, int max_size, const char *format, us_time when);
    //static char *strp(const char *s, const char *format, us_time *when);

  private:
    /*!  \brief Return the OS time in micro-seconds
    */
    static us_time RealTime();  //not effected by time scale

    static struct timeval t;

    us_time requestedDelay;  //in uS
    us_time finishTime; // in uS us_time
    struct timeval start_t;

    static bool fakeTime;
    static double scale;
    static us_time timeBase;
};


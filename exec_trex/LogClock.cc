#include "Clock.hh"
#include <errno.h>
#include <time.h>
#include "LogClock.hh"

#include "LogManager.hh"

namespace TREX {
 /**
   * Real Time Clock
   */
  LogClock::LogClock(double secondsPerTick, bool stats)
    : Clock(secondsPerTick/1000, stats),
      m_gets(0), m_tick(0),
      m_secondsPerTick(secondsPerTick) {
    pthread_mutex_init(&m_lock, NULL);
    m_file = fopen("clock.log", "w");
  }

  void LogClock::start(){
    pthread_create(&m_thread, NULL, threadRunner, this);
  }

  TICK LogClock::getNextTick(){
    pthread_mutex_lock(&m_lock);
    TICK tick = m_tick;
    this->m_gets++;
    pthread_mutex_unlock(&m_lock);
    return tick;
  }

  void* LogClock::threadRunner(void* clk){
    LogClock* This = (LogClock*) clk;

    // Loop forever, sleep for a tick
    double sleepTime = This->m_secondsPerTick;
    while(true){
      Clock::sleep(sleepTime);
      pthread_mutex_lock(&This->m_lock);
      This->advanceTick(This->m_tick);
      fprintf(This->m_file, "%u\n", This->m_gets);
      This->m_gets = 0;
      pthread_mutex_unlock(&This->m_lock);
    }

    return NULL;
  }


  /**
   * Playback clock.
   */
  PlaybackClock::PlaybackClock(double secondsPerTick, bool stats)
    : Clock(0, stats),
      m_gets(0), m_tick(0) {
    m_file = fopen("clock.log", "r");
    if (!m_file) {
      std::cerr << "No clock.log file for playback." << std::endl;
      exit(-1);
    }
  }

  void PlaybackClock::start(){
    fscanf(m_file, "%u\n", &m_gets);
  }

  TICK PlaybackClock::getNextTick() {
    unsigned int timeout = 0;
    while (m_gets == 0 && timeout < 1000) {
      advanceTick(m_tick);
      fscanf(m_file, "%u\n", &m_gets);
      timeout++;
    }
    m_gets--;
    TICK tick = m_tick;
    return tick;
  }
}

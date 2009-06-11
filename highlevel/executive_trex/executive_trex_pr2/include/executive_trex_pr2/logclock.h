#ifndef H_LogClock
#define H_LogClock

#include "AgentClock.hh"
#include "XMLUtils.hh"
#include <pthread.h>
#include <fstream>

#include <ros/time.h>

/**
 * @brief Declaration of clock interface and implementation sub-classes
 */
namespace TREX {
  /**
   * @brief A clock that monitors time on a separate thread and generates updates to the tick and writes time out to "clock.log".
   */
  class LogClock: public Clock {
  public:
    LogClock(double secondsPerTick, bool stats = true);

    virtual ~LogClock();
    /**
     * @brief Will idle till this is called.
     */
    void start();

    /**
     * @brief Retrieve the tick
     */
    TICK getNextTick();

    /**
     */
    virtual double getSecondsPerTick() const {return m_secondsPerTick;}

    /*
     * @brief Monitors elapsed time and increments the tick counter
     */
    static void* threadRunner(void* clk);

  protected:

  private:
    unsigned int m_gets;
    TICK m_tick;
    double m_secondsPerTick;
    pthread_t m_thread;
    pthread_mutex_t m_lock;
    std::ofstream m_file;
    bool m_enabled;
    ros::Time m_start_time;
  };
  /**
   * @brief A clock that reads time out of "clock.log" and generates updates to the tick.
   */
  class PlaybackClock: public Clock {
  public:
    PlaybackClock(bool stats = true);

    virtual ~PlaybackClock();

    /**
     * @brief Will idle till this is called.
     */
    void start();

    /**
     * @brief Retrieve the tick
     */
    TICK getNextTick();

  private:

    unsigned int m_gets;
    TICK m_tick; 
    FILE* m_file;
  };
}

#endif

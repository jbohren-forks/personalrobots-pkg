#ifndef H_LogClock
#define H_LogClock

#include "Clock.hh"
#include "XMLUtils.hh"
#include <pthread.h>

/**
 * @brief Declaration of clock interface and implementation sub-classes
 */
namespace TREX {
  /**
   * @brief A clock that monitors time on a separate thread and generates updates to the tick.
   */
  class LogClock: public Clock {
  public:
    LogClock(double secondsPerTick, bool stats = true);

    /**
     * @brief Will idle till this is called.
     */
    void start();

    /**
     * @brief Retrieve the tick
     */
    TICK getNextTick();

    /**
     * @brief Monitors elapsed time and increments the tick counter
     */
    static void* threadRunner(void* clk);

  private:
    unsigned int m_gets;
    FILE* m_file;
    TICK m_tick;
    double m_secondsPerTick;
    pthread_t m_thread;
    pthread_mutex_t m_lock;
  };
  /**
   * @brief A clock that monitors time on a separate thread and generates updates to the tick.
   */
  class PlaybackClock: public Clock {
  public:
    PlaybackClock(unsigned int finalTick, TiXmlElement* root, bool stats = true);

    /**
     * @brief Will idle till this is called.
     */
    void start();

    /**
     * @brief Retrieve the tick
     */
    TICK getNextTick();
    /**
     * @brief Get user input.
     */
    void consolePopup();
    /**
     * @brief Returns true if we are at the goal tick == should pop up the console.
     */
    bool isAtGoalTick();
    /**
     * @brief Returns true if this system has timed out.
     */
    bool isTimedOut();
  private:
    /**
     * @brief Generate help console popup.
     */
    void printHelp();

    unsigned int m_gets, m_finalTick;
    FILE* m_file;
    TICK m_tick, m_stopTick; 
    TiXmlElement* m_root;
    bool m_timedOut;
  };
}

#endif

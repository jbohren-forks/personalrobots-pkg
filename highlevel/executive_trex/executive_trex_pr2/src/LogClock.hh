#ifndef H_LogClock
#define H_LogClock

#include "Clock.hh"
#include "XMLUtils.hh"
#include <pthread.h>
#include <fstream>

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
    TICK m_tick;
    double m_secondsPerTick;
    pthread_t m_thread;
    pthread_mutex_t m_lock;
    std::ofstream m_file;
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
    TICK m_tick, m_stopTick; 
    TiXmlElement* m_root;
    FILE* m_file;
    bool m_timedOut;
  };
}

#endif

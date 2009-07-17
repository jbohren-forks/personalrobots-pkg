#include "trex_ros/logclock.h"
#include "Agent.hh"
#include "LogManager.hh"
#include "trex_ros/components.h"
#include "Utilities.hh"
#include "ros/time.h"
#include <errno.h>
#include <time.h>
#include <signal.h>

namespace TREX {
 /**
   * Real Time Clock
   */
  LogClock::LogClock(double secondsPerTick, bool stats)
    : Clock(secondsPerTick, stats),
      m_gets(0), m_tick(0),
      m_secondsPerTick(secondsPerTick),
      m_file(LogManager::instance().file_name("clock.log").c_str()),
      m_enabled(false){
    pthread_mutex_init(&m_lock, NULL);
  }

  LogClock::~LogClock() {
    m_enabled = false;
    pthread_join(m_thread, NULL);
    pthread_mutex_destroy(&m_lock);
  }

  void LogClock::start(){
    // Get start time
    m_start_time = ros::Time::now();

    // Enable and spawn thread
    m_enabled = true;
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
    signal(SIGINT,  &TREX::signalHandler);
    signal(SIGTERM, &TREX::signalHandler);
    signal(SIGQUIT, &TREX::signalHandler);
    signal(SIGKILL, &TREX::signalHandler);

    LogClock* This = (LogClock*) clk;

    // Loop forever, sleep for a tick
    double sleepTimePerTick = This->m_secondsPerTick;
    ros::Duration sleepDuration;

    while(This->m_enabled){
      ros::Time::sleepUntil(This->m_start_time + sleepDuration.fromSec(This->m_tick*sleepTimePerTick));

      pthread_mutex_lock(&This->m_lock);
      This->advanceTick(This->m_tick);
      This->m_file << This->m_gets << std::endl;
      This->m_gets = 0;
      pthread_mutex_unlock(&This->m_lock);
    }

    return NULL;
  }

  double LogClock::getSleepDelay() const {
    //ROS_INFO("LOG CLOCK SLEEP DELAY %f", m_secondsPerTick);
    return m_secondsPerTick;
  }

  /**
   * Playback clock.
   */
  PlaybackClock::PlaybackClock(bool stats)
    : Clock(0, stats),
      m_gets(0), m_tick(0) {

    // Open clock log file 
    m_file = fopen(findFile("clock.log").c_str(), "r");
    if (!m_file) {
      std::cerr << "No clock.log file for playback." << std::endl;
      exit(-1);
    }
  }

  PlaybackClock::~PlaybackClock() {
    fclose(m_file);
  }

  void PlaybackClock::start(){
    if (m_file) {
      fscanf(m_file, "%u\n", &m_gets);
    }
  }

  TICK PlaybackClock::getNextTick() {
    // Check if we have used all the gets for this tick
    if(m_gets == 0) {
      // Increment the tick
      advanceTick(m_tick);
      
      // Read the next get count from clock.log
      if (m_file) {
	fscanf(m_file, "%u\n", &m_gets);
      }
    }

    // Decrement the gets left for this tick
    m_gets--;

    return m_tick;
  }
}

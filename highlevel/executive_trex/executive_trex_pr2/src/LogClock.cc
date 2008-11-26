#include "Clock.hh"
#include "LogClock.hh"
#include "Agent.hh"
#include "LogManager.hh"
#include "Components.hh"
#include "Utilities.hh"
#include <errno.h>
#include <time.h>
#include <signal.h>

namespace TREX {
 /**
   * Real Time Clock
   */
  LogClock::LogClock(double secondsPerTick, bool stats)
    : Clock(secondsPerTick/1000, stats),
      m_gets(0), m_tick(0),
      m_secondsPerTick(secondsPerTick) {
    pthread_mutex_init(&m_lock, NULL);
    m_file = fopen(findFile("./latest/clock.log").c_str(), "w");
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
    signal(SIGINT,  &TREX::signalHandler);
    signal(SIGTERM, &TREX::signalHandler);
    signal(SIGQUIT, &TREX::signalHandler);
    signal(SIGKILL, &TREX::signalHandler);

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
  PlaybackClock::PlaybackClock(unsigned int finalTick, EUROPA::TiXmlElement* root, bool stats)
    : Clock(0, stats),
      m_gets(0), m_finalTick(finalTick), m_tick(0), m_stopTick(0), m_root(root), m_timedOut(false) {
    if (m_finalTick < 2) { m_finalTick = 100; }
    m_file = fopen(findFile("clock.log").c_str(), "r");
    if (!m_file) {
      std::cerr << "No clock.log file for playback." << std::endl;
      exit(-1);
    }
  }

  void PlaybackClock::printHelp() {
    std::cout << "Options:" << std::endl;
    std::cout << " Q :- Quit" << std::endl;
    std::cout << " N :- Next" << std::endl;
    std::cout << " G :- Goto <tick> e.g. g100" << std::endl;
    std::cout << " R :- Reload Debug.cfg" << std::endl;
    std::cout << " + :- Enable pattern e.g. '+Agent'" << std::endl;
    std::cout << " - :- Disable pattern e.g. '-Agent'" << std::endl;
    std::cout << " ! :- Disable all debug messages" << std::endl;
    std::cout << " PW :- Start PlanWorks output" << std::endl;
    std::cout << " EPW :- End PlanWorks output" << std::endl;
    std::cout << " H :- Help" << std::endl;
  }

  void PlaybackClock::consolePopup() {
    std::string cmdString;
    bool cmdValid = false;
    std::cout << "At tick " << m_tick << ", H for help." << std::endl;
    while (!cmdValid) {
      std::cout << "> ";
      getline(std::cin, cmdString);
      const char cmd = cmdString.length() == 0 ? 0 : cmdString.at(0);
      if (!cmd) {
	cmdValid = false;
      } else if(cmd == 'H' || cmd == 'h'){
	printHelp();
	cmdValid = false;
      } else if(cmd == 'Q' || cmd == 'q'){
	std::cout << "Goodbye :-)" << std::endl;
	cmdValid = true;
	exit(0);
      } else if(cmd == 'N' || cmd == 'n'){
	m_stopTick = m_tick + 1;
	cmdValid = true;
      } else if(cmd == 'G' || cmd == 'g'){
	std::string tickStr = cmdString.substr(1);
	if (atoi(tickStr.c_str()) > 0) {
	  m_stopTick = (TICK)atoi(tickStr.c_str());
	  cmdValid = true;
	  if (m_tick > m_stopTick) {
	    cmdValid = true;
	    std::cout << "Doing a rewind." << std::endl;
	    TREX::Agent::terminate();
	    std::cout << "Terminated" << std::endl;
	    TREX::Clock::sleep(1);
	    std::cout << "Slept" << std::endl;
	    TREX::Agent::reset();
	    TREX::Agent::cleanupLog();
	    std::cout << "Reseted" << std::endl;


	    m_tick = 0;
	    fclose(m_file);
	    m_file = fopen("clock.log", "r");
	    if (!m_file) {
	      std::cerr << "No clock.log file for playback." << std::endl;
	      exit(-1);
	    }
	    fscanf(m_file, "%u\n", &m_gets);
	    

	    TREX::Clock::sleep(1);
	    std::cout << "Initing" << std::endl;
	    TREX::Agent::initialize(*m_root, *this);
	    std::cout << "Done" << std::endl;
	  } else if (m_tick == m_stopTick) {
	    cmdValid = false;
	    std::cout << "Already at that tick." << std::endl;
	  }
	} else {
	  std::cout << "Please enter a number." << std::endl;
	  cmdValid = false;
	}
      } else if(cmd == 'R' || cmd == 'r'){
	std::ifstream config("Debug.cfg");
	DebugMessage::readConfigFile(config);
	cmdValid = false;
      } else if(cmdString == "PW"){
	DebugMessage::enableMatchingMsgs("", "PlanWorks");
      } else if(cmdString == "EPW"){
	DebugMessage::disableMatchingMsgs("", "PlanWorks");
      } else if(cmd == '+'){
	std::string pattern = cmdString.substr(1);
	DebugMessage::enableMatchingMsgs("", pattern);
	cmdValid = false;
      } else if(cmd == '-'){
	std::string pattern = cmdString.substr(1);
	DebugMessage::disableMatchingMsgs("", pattern);
	cmdValid = false;
      } else if(cmd == '!'){
	DebugMessage::disableAll();
	cmdValid = false;
      } else {
	cmdValid = false;
      }
    }
    

  }


  void PlaybackClock::start(){
    fscanf(m_file, "%u\n", &m_gets);
    printHelp();
    consolePopup();
  }

  TICK PlaybackClock::getNextTick() {
    unsigned int timeout = 0;
    if (m_tick > m_stopTick) {
      return m_tick;
    }

    while (m_gets == 0 && timeout <= m_finalTick) {
      fscanf(m_file, "%u\n", &m_gets);
      timeout++;
      m_tick++;
    }
    if (timeout >= m_finalTick) {
      m_timedOut = true;
    }
    m_gets--;

    TICK tick = m_tick;
    return tick;
  }

  bool PlaybackClock::isAtGoalTick() {
    return m_tick == m_stopTick;
  }
  bool PlaybackClock::isTimedOut() {
    return m_timedOut;
  }
}

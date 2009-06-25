
#ifndef H_EXECUTIVE
#define H_EXECUTIVE

// roscpp
#include <ros/ros.h>

// TREX
#include "AgentClock.hh"
#include "Agent.hh"
#include "Debug.hh"
#include "LogManager.hh"
#include "trex_ros/logger.h"


int trexMain(int argc, char** argv);

namespace TREX{

  class Executive;
  typedef EUROPA::Id<Executive> ExecutiveId;


  class Executive
  {
  public:

    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static ExecutiveId request(bool playback = false, bool hyper = false);

    /**
     * Releases a reference to the singleton.
     */
    static void release();

    Executive(bool playback, bool hyper);

    ~Executive();

    /**
     * @brief Main execution leak
     */
    void run();
    /*
    template<class T>
    void registerPublisher(const std::string &topic, size_t max_queue){
      ros::Node::instance()->advertise<T>(topic, max_queue);
    }

    template<class M, class T>
    void registerSubscriber(const std::string &_topic, M &_msg, void (T::*fp)(), T* obj, int max_queue){
      ros::Node::instance()->subscribe(_topic, _msg, fp, obj, max_queue);
    }

    template<class M>
    void publishMsg(const std::string &_topic, M &msg){
      ros::Node::instance()->publish(_topic, msg);
    }
    */

    /**
     * @brief Initializes interactive mode.
     */
    void interactiveInit();

    /**
     * @brief Step the executive agent by one step
     */
    bool step();

    /**
     * @brief Allow a reset of the current instance - deallocate it.
     */
    void reset();

  private:

    /**
     * Adds a reference
     */
    void addRef();
    /**
     * Deletes a reference.
     */
    bool decRef();

    /**
     * A watchdog loop to keep pumping ping messages to a watchdog
     */
    void watchDogLoop();

    static ExecutiveId s_id;
    ExecutiveId m_id;
    unsigned int m_refCount;
    double watchDogCycleTime_; /*!< Duration of sleep interval between pings */

    TREX::Clock* agent_clock_; /*!< The clock used to run the agent */
    std::ofstream debug_file_; /*!< TREX debug output sent here */
    TREX::LoggerId logger_; /*!< handles TREX logging */
    TiXmlElement* input_xml_root_; /*!< Root to xml file used by all */
    bool playback_; /*! Will be true if node running off a playback log */
    bool warp_; /*! Will be true if fast playback */
    bool hyper_; /*! Will be true if fast test run */
  };
}


#endif
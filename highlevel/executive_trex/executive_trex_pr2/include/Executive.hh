#ifndef H_EXECUTIVE
#define H_EXECUTIVE

// roscpp
#include <ros/node.h>

// TREX
#include "Clock.hh"
#include "Agent.hh"
#include "Debug.hh"
#include "LogManager.hh"
#include "Logger.hh"

namespace TREX{
  class Executive;
  typedef EUROPA::Id<Executive> ExecutiveId;


  class Executive : public ros::node
  {
  public:

    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static ExecutiveId request();

    /**
     * Releases a reference to the singleton.
     */
    static void release();    

    Executive();

    ~Executive();

    /**
     * @brief Main execution leak
     */
    void run();

    template<class T> 
    void registerPublisher(const std::string &topic, size_t max_queue){
      ros::node::advertise<T>(topic, max_queue);
    }

    template<class M, class T>
    void registerSubscriber(const std::string &_topic, M &_msg, void (T::*fp)(), T* obj, int max_queue){
      ros::node::subscribe(_topic, _msg, fp, obj, max_queue);
    }
    
    template<class M>
    void publishMsg(const std::string &_topic, M &msg){
      ros::node::publish<M>(_topic, msg);
    }
    
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
  };
}


#endif

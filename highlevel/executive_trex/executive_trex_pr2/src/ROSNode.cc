#include <signal.h>

#include "ROSNode.hh"

//NDDL includes
#include "Nddl.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Utilities.hh"

// TREX Includes 
#include "Agent.hh"
#include "Debug.hh"

#include "LogManager.hh"

// Requirements for watchdog
#include <highlevel_controllers/Ping.h>
#include <sys/time.h>
#include <rosthread/member_thread.h>

namespace TREX {

  ROSNodeId ROSNode::s_id;

  /**
   * ROSNode class implementation.
   */


  /**
   * @brief Singleton accessor
   */
  ROSNodeId ROSNode::request(){
    if(s_id == ROSNodeId::noId()){
      int argc = 0;
      ros::init(argc, NULL);
      new ROSNode();
    }
    s_id->addRef();
    return s_id;
  }

  /**
   * @brief Sets up publish and subscribe topics for this node to integrate it
   * at the top to publish Execution Status updates and at the bottom to dispatch 
   * commands to the RCS and receive updates
   */
  ROSNode::ROSNode() : ros::node("trex"), m_id(this), watchDogCycleTime_(0.1)
  {
    s_id = m_id; 
    m_refCount = 0;

    // Bind the watchdog loop sleep time from input controller frequency
    double ping_frequency(10.0);
    param("executive/ping_frequency", ping_frequency, ping_frequency);
    if(ping_frequency > 0)
      watchDogCycleTime_ = 1 / ping_frequency;
  
    ros::node::advertise<highlevel_controllers::Ping>("executive/ping", 1);

    // Start watchdog on a separate thread
    ros::thread::member_thread::startMemberFunctionThread(this, &ROSNode::watchDogLoop); 

    ROS_INFO("ROSNode created.\n");
  }

  void ROSNode::watchDogLoop(){
    highlevel_controllers::Ping pingMsg;
    while(!Agent::terminated()){
      publish<highlevel_controllers::Ping>("executive/ping", pingMsg);
      usleep((unsigned int) rint(watchDogCycleTime_ * 1e6));
    }
  }

  void ROSNode::release() {
    if (ROSNode::s_id->decRef()) {
      ROS_INFO("Terminating ROS node.\n");
      ROSNode::s_id->shutdown();
      delete (ROSNode*)s_id;
      s_id = ROSNodeId::noId();
    }
  }

  ROSNode::~ROSNode() {
    m_id.remove();
  }

  void ROSNode::addRef() {
    m_refCount++;
  }

  bool ROSNode::decRef() {
    m_refCount--;
    return m_refCount == 0;
  }
}

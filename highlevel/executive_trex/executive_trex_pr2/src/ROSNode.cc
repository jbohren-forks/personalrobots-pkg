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
  ROSNode::ROSNode() : ros::node("trex"), m_id(this)
  {

    debugMsg("ROSNode:Create", "ROSNode created.");
    s_id = m_id; 
    m_refCount = 0;
    debugMsg("ROSNode:Create", "Done with ROSNODE::constructor.");
  }


  void ROSNode::release() {
    if (ROSNode::s_id->decRef()) {
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

#include "ROSAdapter.hh"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"
#include "ROSNode.hh"
#include "Logger.hh"

namespace TREX {

  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, configData), m_initialized(false){
    m_node = ROSNode::request();
  }

  void ROSAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    TREX::Adapter::handleInit(initialTick, serversByTimeline, observer);

    registerPublishers();
    registerSubscribers();

    // Wait till we get a message before starting the agent
    while(!isInitialized() && m_node->ok()){
      debugMsg("ROSAdapter:Create", "Waiting...");
      sleep(1);
    }
  }

  ROSAdapter::~ROSAdapter() {
    m_node->release();
  }

  bool ROSAdapter::synchronize(){
    debugMsg("ROSAdapter:synchronize", nameString() << "Checking..");

    std::vector<Observation*> obsBuffer;

    // Derived class will populate actual observations
    m_node->lock();
    getObservations(obsBuffer);
    m_node->unlock();

    for(unsigned int i = 0; i<obsBuffer.size(); i++){
      Observation* obs = obsBuffer[i];
      sendNotify(*obs);
      delete obs;
    }

    return true;
  }

  void ROSAdapter::handleNextTick(){}

  void ROSAdapter::handleCallback(){
    debugMsg("ROSAdapter:handleCallback", nameString() << "Update received");
    m_node->lock();
    m_initialized = true;
    m_node->unlock();
  }


  bool ROSAdapter::isInitialized() const {
    return m_initialized;
  }
}

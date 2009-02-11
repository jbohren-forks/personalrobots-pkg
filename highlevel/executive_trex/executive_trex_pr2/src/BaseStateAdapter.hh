#ifndef H_BaseStateAdapter
#define H_BaseStateAdapter

#include "ROSAdapter.hh"

// State Update messages
#include <deprecated_msgs/RobotBase2DOdom.h>

namespace TREX {

  class BaseStateAdapter: public ROSAdapter {
  public:
    BaseStateAdapter(const LabelStr& agentName, const TiXmlElement& configData): ROSAdapter(agentName, configData) {}
    virtual ~BaseStateAdapter(){}

  protected:
    // Message and message handler for base odometry observations
    virtual void handleCallback();
    virtual void registerSubscribers();
    virtual void getObservations(std::vector<Observation*>& obsBuffer);
    deprecated_msgs::RobotBase2DOdom m_msgRobotBase2DOdom;
  };
}
#endif

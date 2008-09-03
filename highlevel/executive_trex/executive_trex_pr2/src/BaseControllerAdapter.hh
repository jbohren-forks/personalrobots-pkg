#ifndef H_BaseControllerAdapter
#define H_BaseControllerAdapter

#include "ROSAdapter.hh"

// State Update messages
#include <std_msgs/Planner2DState.h>

namespace TREX {

  class BaseControllerAdapter: public ROSAdapter {
  public:
    /**
     * @brief The last reported state of the controller
     */
    enum State {UNDEFINED = 0,
		INACTIVE,
		ACTIVE};

    BaseControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData);
    virtual ~BaseControllerAdapter(){}

  private:
    // Message and message handler for base odometry observations
    virtual void handleCallback();
    virtual void dispatch(const TokenId& goal);
    virtual void registerSubscribers();
    virtual void registerPublishers();
    virtual void getObservations(std::vector<Observation*>& obsBuffer);
    std_msgs::Planner2DState m_msgPlanner2DState;
    State m_state;
    int m_lastPublished;
    TICK m_lastUpdated;
  };
}
#endif

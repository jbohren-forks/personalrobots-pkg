#ifndef H_BaseControllerAdapter
#define H_BaseControllerAdapter

#include "ROSAdapter.hh"

// State Update messages
#include <robot_msgs/Planner2DState.h>

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
    void handleCallback();
    void registerSubscribers();
    void registerPublishers();
    Observation* getObservation();
    void dispatchRequest(const TokenId& goal, bool enabled);

    std_msgs::Planner2DState m_msgPlanner2DState;
    State m_state;
    int m_lastPublished;
    TICK m_lastUpdated;
  };
}
#endif

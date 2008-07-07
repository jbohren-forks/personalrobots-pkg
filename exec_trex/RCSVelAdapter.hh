#ifndef H_RCSVelAdapter
#define H_RCSVelAdapter

#include "Adapter.hh"
#include "ROSNode.hh"

namespace TREX {

  class RCSVelAdapter: public Adapter {
  public:
    RCSVelAdapter(const LabelStr& agentName, const TiXmlElement& configData);
    virtual ~RCSVelAdapter();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    void handleRequest(const TokenId& goal);

  private:
    ObserverId m_observer; /*! Apointer for publishing observations to the agent. */
    ROSNodeId m_node; /*! The ROS node. */
  };
}
#endif

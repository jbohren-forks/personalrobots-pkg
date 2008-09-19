#ifndef H_ROSAdapter
#define H_ROSAdapter

#include "Adapter.hh"
#include "ROSNode.hh"

namespace TREX {

  class ROSAdapter: public Adapter {
  public:
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData);
    virtual ~ROSAdapter();

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

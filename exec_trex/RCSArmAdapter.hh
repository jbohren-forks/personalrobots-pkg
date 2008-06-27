#ifndef H_RCSVelAdapter
#define H_RCSVelAdapter

//#include "ExecDefs.hh"
#include "Adapter.hh"
#include "ROSNode.hh"

namespace TREX {

  class RCSArmAdapter: public Adapter {
  public:
    RCSArmAdapter(const LabelStr& agentName, const TiXmlElement& configData);
    ~RCSArmAdapter();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    void handleRequest(const TokenId& goal);

  private:
    ObserverId m_observer;
    ROSNodeId m_node; /*! The ROS node. */
  };
}
#endif

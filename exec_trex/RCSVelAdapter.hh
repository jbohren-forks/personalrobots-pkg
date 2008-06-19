#ifndef H_RCSVelAdapter
#define H_RCSVelAdapter

#include "ExecDefs.hh"
#include "Adapter.hh"

namespace TREX {

  class RCSVelAdapter: public Adapter {
  public:
    RCSVelAdapter(const LabelStr& agentName, const TiXmlElement& configData);

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    void handleRequest(const TokenId& goal);

  private:
    ObserverId m_observer;
  };
}
#endif

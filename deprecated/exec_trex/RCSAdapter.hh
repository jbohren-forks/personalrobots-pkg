#ifndef H_RCSAdapter
#define H_RCSAdapter

#include "ExecDefs.hh"
#include "Adapter.hh"

namespace TREX {

  class RCSAdapter: public Adapter {
  public:
    RCSAdapter(const LabelStr& agentName, const TiXmlElement& configData);

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    void handleRequest(const TokenId& goal);

  private:
    ObserverId m_observer;
  };
}
#endif

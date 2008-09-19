#ifndef H_RCSVelAdapterPlayback
#define H_RCSVelAdapterPlayback

//#include "ExecDefs.hh"
#include "Playback.hh"
#include "Adapter.hh"

namespace TREX {

  class RCSVelAdapterPlayback : public Adapter {
  public:
    RCSVelAdapterPlayback(const LabelStr& agentName, const TiXmlElement& configData);
    ~RCSVelAdapterPlayback();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    void handleRequest(const TokenId& goal);

  private:
    void createObservationsFromXML(std::vector<EUROPA::TiXmlElement*> &nodes);
    PlaybackId m_playback; /*! Playback system. */
    ObserverId m_observer; /*! Apointer for publishing observations to the agent. */
    TiXmlElement* logRoot; /*! XML Root */
  };
}
#endif

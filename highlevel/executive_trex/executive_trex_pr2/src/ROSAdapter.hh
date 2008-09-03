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
    bool isInitialized() const;

    bool m_initialized; /*!< Hook to flag if expected ros initialization messages have arrived */


  protected:
    virtual void handleCallback();
    virtual void dispatch(const TokenId& goal){}
    virtual void registerSubscribers() {}
    virtual void registerPublishers() {}
    virtual void getObservations(std::vector<Observation*>& obsBuffer) = 0;
    ROSNodeId m_node; /*! The ROS node. */
  };
}
#endif

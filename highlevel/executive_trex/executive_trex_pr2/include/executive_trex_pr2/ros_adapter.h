#ifndef H_ROSAdapter
#define H_ROSAdapter

#include "executive_trex_pr2/executive.h"
#include "Adapter.hh"
#include "Debug.hh"

namespace TREX {

  class ROSAdapter: public Adapter {
  public:
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead);
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic);

    virtual ~ROSAdapter();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

  private:
    bool isInitialized() const;

    bool handleRequest(const TokenId& goal);

    void handleRecall(const TokenId& goal);

    void commonInit(const TiXmlElement& configData);

    bool m_initialized; /*!< Hook to flag if expected ros initialization messages have arrived */
    std::vector<std::string> nddlNames_;
    std::vector<std::string> rosNames_;

  protected:
    static unsigned int QUEUE_MAX(){return 10;}
    virtual void handleCallback();
    virtual void registerSubscribers() {}
    virtual void registerPublishers() {}
    virtual Observation* getObservation() = 0;
    virtual bool dispatchRequest(const TokenId& goal, bool enabled){return true;}

    const std::vector<std::string>& nddlNames() const {return nddlNames_;}
    const std::vector<std::string>& rosNames() const {return rosNames_;}

    /**
     * @brief Helper method to obtain an index for a given ros name.
     * @param The rosName to look for
     * @param The index to fill if found
     * @return True if found, else false
     */
    bool rosIndex(const std::string& rosName, unsigned int& ind) const;

  
  
    ExecutiveId m_node; /*! The node. */
    const std::string timelineName;
    const std::string timelineType;
    const std::string stateTopic;
  };
}
#endif

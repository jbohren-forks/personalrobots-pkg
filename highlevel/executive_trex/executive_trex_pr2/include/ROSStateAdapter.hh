#ifndef H_ROSStateAdapter
#define H_ROSStateAdapter

#include "ROSAdapter.hh"

namespace TREX {

  template <class S> class ROSStateAdapter: public ROSAdapter {
  public:
    ROSStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSAdapter(agentName, configData, 0), predicate(timelineType + ".Holds"){
    }

    virtual ~ROSStateAdapter(){}

  protected:
    virtual void handleCallback(){
      ROSAdapter::handleCallback();
    }

    Observation* getObservation(){
      ObservationByValue* obs = new ObservationByValue(timelineName, predicate);
      stateMsg.lock();
      fillObservationParameters(obs);
      stateMsg.unlock();
      return obs;
    }

    virtual void fillObservationParameters(ObservationByValue* obs) = 0;

    S stateMsg; /*!< The ros message on which we get observation updates */

  private:
    void registerSubscribers() {
      debugMsg("ROS", "Registering subscriber for " << timelineName << " on topic " << stateTopic);
      m_node->registerSubscriber(stateTopic, stateMsg, &TREX::ROSStateAdapter<S>::handleCallback, this, QUEUE_MAX());
    }

    const std::string predicate;
  };
}
#endif

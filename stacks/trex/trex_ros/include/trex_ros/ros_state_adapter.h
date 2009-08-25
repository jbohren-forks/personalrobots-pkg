#ifndef H_ROSStateAdapter
#define H_ROSStateAdapter

#include "trex_ros/ros_adapter.h"
#include "ros/ros.h"
#include "Domains.hh"

namespace TREX {

  template <class S> class ROSStateAdapter: public ROSAdapter {
  public:
    ROSStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSAdapter(agentName, configData, 0), predicate(timelineType + ".Holds"){
    }

    virtual ~ROSStateAdapter(){}

  protected:
    virtual void handleCallback(const boost::shared_ptr<const S> msg){
      stateMsgLock.lock();
      stateMsg = *msg;
      stateMsgLock.unlock();
      ROSAdapter::handleCallback();
    }

    Observation* getObservation(){
      ObservationByValue* obs = new ObservationByValue(timelineName, predicate);
      fillObservationParameters(obs);
      return obs;
    }

    virtual void fillObservationParameters(ObservationByValue* obs) = 0;

    S stateMsg; /*!< The ros message on which we get observation updates */
    boost::mutex stateMsgLock;
  private:
    ros::NodeHandle _node;
    ros::Subscriber _state_sub;
    void registerSubscribers() {
      TREX_INFO("ros:debug", nameString() << "Registering subscriber for " << timelineName << " on topic " << stateTopic);
      _state_sub = _node.subscribe<S>(stateTopic, QUEUE_MAX(), boost::bind(&TREX::ROSStateAdapter<S>::handleCallback, this, _1));
    }

    const std::string predicate;
  };
}
#endif

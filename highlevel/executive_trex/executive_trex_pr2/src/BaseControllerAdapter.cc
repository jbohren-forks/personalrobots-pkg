#include "BaseControllerAdapter.hh"
#include "IntervalDomain.hh"
#include "Token.hh"
#include <std_msgs/Planner2DGoal.h>

namespace TREX {
  BaseControllerAdapter::BaseControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : ROSAdapter(agentName, configData), m_state(BaseControllerAdapter::UNDEFINED), m_lastPublished(-1), m_lastUpdated(0) {}

  void BaseControllerAdapter::handleCallback(){
    ROSAdapter::handleCallback();

    if(m_msgPlanner2DState.active && m_state != BaseControllerAdapter::ACTIVE){
      m_state = BaseControllerAdapter::ACTIVE;
      m_lastUpdated = getCurrentTick();
    }
    else if(!m_msgPlanner2DState.active && m_state != BaseControllerAdapter::INACTIVE){
      m_state = BaseControllerAdapter::INACTIVE;
      m_lastUpdated = getCurrentTick();
    }
  }

  void BaseControllerAdapter::registerPublishers(){
    static const size_t QUEUE_MAX(1000);
    m_node->registerPublisher<std_msgs::Planner2DGoal>("goal", QUEUE_MAX);
  }

  void BaseControllerAdapter::registerSubscribers(){
    static const size_t QUEUE_MAX(1000);
    m_node->registerSubscriber("state", m_msgPlanner2DState, &BaseControllerAdapter::handleCallback, this, QUEUE_MAX);
  }

  /**
   * @brief This method assumes the lock has been aquired.
   */
  void BaseControllerAdapter::getObservations(std::vector<Observation*>& obsBuffer){
    // Nothing to do if we published for the last update
    if(((int) m_lastUpdated) == m_lastPublished)
      return;

    if(m_state == BaseControllerAdapter::UNDEFINED)
      throw "BaseControllerAdapter: Tried to get an observation for adapter with no initial state set yet.";

    double x, y, th;
    ObservationByValue* obs = NULL;

    if(m_state == BaseControllerAdapter::INACTIVE){
      x = m_msgPlanner2DState.pos.x;
      y = m_msgPlanner2DState.pos.y;
      th= m_msgPlanner2DState.pos.th;
      obs = new ObservationByValue("moveBase", "MoveBase.Inactive");
    }
    else {
      x = m_msgPlanner2DState.goal.x;
      y = m_msgPlanner2DState.goal.y;
      th= m_msgPlanner2DState.goal.th;
      obs = new ObservationByValue("moveBase", "MoveBase.Active");
    }

    obs->push_back("x", new IntervalDomain(x));
    obs->push_back("y", new IntervalDomain(y));
    obs->push_back("th",new IntervalDomain(th));
    obsBuffer.push_back(obs);
    m_lastPublished = m_lastUpdated;
  }

  void BaseControllerAdapter::dispatch(const TokenId& goal){
    static const LabelStr MOVE_BASE_ACTIVE("MoveBase.Active");
    if(goal->getPredicateName() == MOVE_BASE_ACTIVE){
      const IntervalDomain& x = goal->getVariable("x")->lastDomain();
      const IntervalDomain& y = goal->getVariable("y")->lastDomain();
      const IntervalDomain& th = goal->getVariable("th")->lastDomain();

      assertTrue(x.isSingleton() && y.isSingleton() && th.isSingleton(), "Values for dispatch are not bound");

      std_msgs::Planner2DGoal msgGoal;
      msgGoal.goal.x = x.getSingletonValue();
      msgGoal.goal.y = y.getSingletonValue();
      msgGoal.goal.th= th.getSingletonValue();
      msgGoal.enable = 1;
      m_node->publishMsg<std_msgs::Planner2DGoal>("goal", msgGoal);
    }
  }

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseControllerAdapter> l_BaseControllerAdapter_Factory("BaseControllerAdapter");
}

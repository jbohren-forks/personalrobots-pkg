#include "BaseStateAdapter.hh"
#include "IntervalDomain.hh"

namespace TREX {

  void BaseStateAdapter::handleCallback(){
    ROSAdapter::handleCallback();
  }

  void BaseStateAdapter::registerSubscribers(){
    m_node->registerSubscriber("localizedpose", m_msgRobotBase2DOdom, &BaseStateAdapter::handleCallback, this, QUEUE_MAX());
  }

  /**
   * @brief This method assumes the lock has been aquired.
   */
  void BaseStateAdapter::getObservations(std::vector<Observation*>& obsBuffer){
    double x, y, th;
    x = m_msgRobotBase2DOdom.pos.x;
    y = m_msgRobotBase2DOdom.pos.y;
    th= m_msgRobotBase2DOdom.pos.th;
    ObservationByValue* obs = new ObservationByValue("baseState", "BaseState.Holds");
    obs->push_back("x", new IntervalDomain(x));
    obs->push_back("y", new IntervalDomain(y));
    obs->push_back("th",new IntervalDomain(th));
    obsBuffer.push_back(obs);
  }

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseStateAdapter> l_BaseStateAdapter_Factory("BaseStateAdapter");
}

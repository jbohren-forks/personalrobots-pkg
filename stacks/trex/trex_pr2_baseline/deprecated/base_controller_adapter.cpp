#include "ROSControllerAdapter.hh"
#include "Domains.hh"
#include "Token.hh"
#include <robot_actions/Pose2D.h>
#include <robot_actions/MoveBaseState.h>

namespace TREX {

  class BaseControllerAdapter: public ROSControllerAdapter<robot_actions::MoveBaseState, robot_actions::Pose2D> {
  public:

    BaseControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSControllerAdapter<robot_actions::MoveBaseState, robot_actions::Pose2D>(agentName, configData){
    }

    virtual ~BaseControllerAdapter(){}

  protected:

    void fillActiveObservationParameters(ObservationByValue* obs){
      obs->push_back("x", new IntervalDomain(stateMsg.goal.x));
      obs->push_back("y", new IntervalDomain(stateMsg.goal.y));
      obs->push_back("th", new IntervalDomain(stateMsg.goal.th));
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
      obs->push_back("x", new IntervalDomain(stateMsg.feedback.x));
      obs->push_back("y", new IntervalDomain(stateMsg.feedback.y));
      obs->push_back("th", new IntervalDomain(stateMsg.feedback.th));
    }

    void fillRequestParameters(robot_actions::Pose2D& goalMsg, const TokenId& goalToken){
      const IntervalDomain& x = goalToken->getVariable("x")->lastDomain();
      const IntervalDomain& y = goalToken->getVariable("y")->lastDomain();
      const IntervalDomain& th = goalToken->getVariable("th")->lastDomain();

      goalMsg.header.frame_id = "/map";
      goalMsg.x = (x.isSingleton()  ? x.getSingletonValue() : (x.getLowerBound() + x.getUpperBound()) / 2);
      goalMsg.y = (y.isSingleton()  ? y.getSingletonValue() : (y.getLowerBound() + y.getUpperBound()) / 2);
      goalMsg.th = (th.isSingleton()  ? th.getSingletonValue() : (th.getLowerBound() + th.getUpperBound()) / 2);
    }

  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseControllerAdapter> l_BaseControllerAdapter_Factory("BaseControllerAdapter");
}

#include "ROSControllerAdapter.hh"
#include "Domains.hh"
#include "Token.hh"
#include <robot_actions/RechargeGoal.h>
#include <robot_actions/RechargeState.h>

namespace TREX {

  class RechargeControllerAdapter: public ROSControllerAdapter<robot_actions::RechargeState, robot_actions::RechargeGoal> {
  public:

    RechargeControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSControllerAdapter<robot_actions::RechargeState, robot_actions::RechargeGoal>(agentName, configData){
    }

    virtual ~RechargeControllerAdapter(){}

  protected:
    

    void fillActiveObservationParameters(ObservationByValue* obs){
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
    }

    void fillRequestParameters(robot_actions::RechargeGoal& goalMsg, const TokenId& goalToken){
      const IntervalDomain& x = goalToken->getVariable("x")->lastDomain();
      const IntervalDomain& y = goalToken->getVariable("y")->lastDomain();
      const IntervalDomain& th = goalToken->getVariable("th")->lastDomain();
      const IntervalDomain& recharge_level = goalToken->getVariable("recharge_level")->lastDomain();

      goalMsg.pose.position.x = (x.isSingleton()  ? x.getSingletonValue() : (x.getLowerBound() + x.getUpperBound()) / 2);
      goalMsg.pose.position.y = (y.isSingleton()  ? y.getSingletonValue() : (y.getLowerBound() + y.getUpperBound()) / 2);
      goalMsg.pose.orientation.w = (th.isSingleton()  ? th.getSingletonValue() : (th.getLowerBound() + th.getUpperBound()) / 2);
      goalMsg.recharge_level = (recharge_level.isSingleton()  ? recharge_level.getSingletonValue() : (recharge_level.getLowerBound() + recharge_level.getUpperBound()) / 2);
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<RechargeControllerAdapter> RechargeControllerAdapter_Factory("RechargeControllerAdapter");
}

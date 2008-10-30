#include "ROSControllerAdapter.hh"
#include "IntervalDomain.hh"
#include "BoolDomain.hh"
#include "Token.hh"
#include <highlevel_controllers/RechargeGoal.h>
#include <highlevel_controllers/RechargeState.h>

namespace TREX {

  class RechargeControllerAdapter: public ROSControllerAdapter<highlevel_controllers::RechargeState, highlevel_controllers::RechargeGoal> {
  public:

    RechargeControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSControllerAdapter<highlevel_controllers::RechargeState, highlevel_controllers::RechargeGoal>(agentName, configData){
    }

    virtual ~RechargeControllerAdapter(){}

  protected:
    

    void fillActiveObservationParameters(ObservationByValue* obs){
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
    }

    void fillRequestParameters(highlevel_controllers::RechargeGoal& goalMsg, const TokenId& goalToken){
      const IntervalDomain& x = goalToken->getVariable("x")->lastDomain();
      const IntervalDomain& y = goalToken->getVariable("y")->lastDomain();
      const IntervalDomain& th = goalToken->getVariable("th")->lastDomain();
      const IntervalDomain& recharge_level = goalToken->getVariable("recharge_level")->lastDomain();

      goalMsg.pose.x = (x.isSingleton()  ? x.getSingletonValue() : (x.getLowerBound() + x.getUpperBound()) / 2);
      goalMsg.pose.y = (y.isSingleton()  ? y.getSingletonValue() : (y.getLowerBound() + y.getUpperBound()) / 2);
      goalMsg.pose.th = (th.isSingleton()  ? th.getSingletonValue() : (th.getLowerBound() + th.getUpperBound()) / 2);
      goalMsg.recharge_level = (recharge_level.isSingleton()  ? recharge_level.getSingletonValue() : (recharge_level.getLowerBound() + recharge_level.getUpperBound()) / 2);
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<RechargeControllerAdapter> RechargeControllerAdapter_Factory("RechargeControllerAdapter");
}

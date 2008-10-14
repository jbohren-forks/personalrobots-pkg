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
      obs->push_back("plugIn", new IntervalDomain(stateMsg.desPlugIn));
      obs->push_back("x", new IntervalDomain(stateMsg.goal.x));
      obs->push_back("y", new IntervalDomain(stateMsg.goal.y));
      obs->push_back("th", new IntervalDomain(stateMsg.goal.th));
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
      obs->push_back("plugIn", new IntervalDomain(stateMsg.plugIn));
    }

    void fillRequestParameters(highlevel_controllers::RechargeGoal& goalMsg, const TokenId& goalToken){
      const BoolDomain& plugIn = goalToken->getVariable("plugIn")->lastDomain();
      const IntervalDomain& x = goalToken->getVariable("x")->lastDomain();
      const IntervalDomain& y = goalToken->getVariable("y")->lastDomain();
      const IntervalDomain& th = goalToken->getVariable("th")->lastDomain();

      goalMsg.plugIn = (bool) (plugIn.isSingleton()  ? plugIn.getSingletonValue() : 0);
      goalMsg.goal.x = (x.isSingleton()  ? x.getSingletonValue() : (x.getLowerBound() + x.getUpperBound()) / 2);
      goalMsg.goal.y = (y.isSingleton()  ? y.getSingletonValue() : (y.getLowerBound() + y.getUpperBound()) / 2);
      goalMsg.goal.th = (th.isSingleton()  ? th.getSingletonValue() : (th.getLowerBound() + th.getUpperBound()) / 2);
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<RechargeControllerAdapter> RechargeControllerAdapter_Factory("RechargeControllerAdapter");
}

#include "ros_action_adapter.h"
#include "Domains.hh"
#include "Token.hh"
#include <std_msgs/String.h>
#include <robot_actions/ShellCommandState.h>

namespace TREX {

  class ShellCommandAdapter: public ROSActionAdapter<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String> {
  public:

    ShellCommandAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String>(agentName, configData){
    }

    virtual ~ShellCommandAdapter(){}

  protected:
    
    virtual void fillActiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){
      obs->push_back("request", new StringDomain(LabelStr(msg.data), "string"));
    }

    virtual void fillInactiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){ 
      obs->push_back("response", new StringDomain(LabelStr(msg.data), "string"));
    }

    void fillDispatchParameters(std_msgs::String& msg, const TokenId& goalToken){
      const StringDomain& dom = static_cast<const StringDomain&>(goalToken->getVariable("request")->lastDomain());
      msg.data = (dom.isSingleton() ? LabelStr(dom.getSingletonValue()).toString() : "\"NULL\"");
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<ShellCommandAdapter> ShellCommandAdapter_Factory("ShellCommandAdapter");
}

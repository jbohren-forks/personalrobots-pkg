#include "trex_ros/ros_state_adapter.h"
#include "Domains.hh"
#include <pr2_msgs/PowerState.h>

namespace TREX {

  class BatteryStateAdapter: public ROSStateAdapter<pr2_msgs::PowerState> {
  public:
    BatteryStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<pr2_msgs::PowerState> ( agentName, configData) {
    }

    virtual ~BatteryStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){
      obs->push_back("time_remaining", new IntervalDomain(stateMsg.time_remaining));
      obs->push_back("AC_present", new IntervalDomain(stateMsg.AC_present));
      obs->push_back("power_consumption",new IntervalDomain(stateMsg.power_consumption));
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BatteryStateAdapter> l_BatteryStateAdapter_Factory("BatteryStateAdapter");
}

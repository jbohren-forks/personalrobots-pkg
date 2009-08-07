#include "trex_ros/ros_state_adapter.h"
#include "Domains.hh"
#include <pr2_msgs/BatteryState.h>

namespace TREX {

  class BatteryStateAdapter: public ROSStateAdapter<pr2_msgs::BatteryState> {
  public:
    BatteryStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<pr2_msgs::BatteryState> ( agentName, configData) {
    }

    virtual ~BatteryStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){
      obs->push_back("energy_capacity", new IntervalDomain(stateMsg.energy_capacity));
      obs->push_back("energy_remaining", new IntervalDomain(std::min(stateMsg.energy_remaining, stateMsg.energy_capacity)));
      obs->push_back("power_consumption",new IntervalDomain(stateMsg.power_consumption));
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BatteryStateAdapter> l_BatteryStateAdapter_Factory("BatteryStateAdapter");
}

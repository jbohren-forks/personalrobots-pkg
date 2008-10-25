#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <robot_msgs/BatteryState.h>

namespace TREX {

  class BatteryStateAdapter: public ROSStateAdapter<robot_msgs::BatteryState> {
  public:
    BatteryStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<robot_msgs::BatteryState> ( agentName, configData) {
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

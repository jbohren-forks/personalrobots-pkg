#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <std_msgs/PR2Arm.h>

namespace TREX {

  class ArmStateAdapter: public ROSStateAdapter<std_msgs::PR2Arm> {
  public:
    ArmStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<std_msgs::PR2Arm> ( agentName, configData) {
    }

    virtual ~ArmStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){
      obs->push_back("turretAngle", new IntervalDomain(stateMsg.turretAngle));
      obs->push_back("shoulderLiftAngle", new IntervalDomain(stateMsg.shoulderLiftAngle));
      obs->push_back("upperarmRollAngle", new IntervalDomain(stateMsg.upperarmRollAngle));
      obs->push_back("elbowAngle", new IntervalDomain(stateMsg.elbowAngle));
      obs->push_back("forearmRollAngle", new IntervalDomain(stateMsg.forearmRollAngle));
      obs->push_back("wristPitchAngle", new IntervalDomain(stateMsg.wristPitchAngle));
      obs->push_back("wristRollAngle", new IntervalDomain(stateMsg.wristRollAngle));
      obs->push_back("gripperForceCmd", new IntervalDomain(stateMsg.gripperForceCmd));
      obs->push_back("gripperGapCmd", new IntervalDomain(stateMsg.gripperGapCmd));
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<ArmStateAdapter> l_ArmStateAdapter_Factory("ArmStateAdapter");
}

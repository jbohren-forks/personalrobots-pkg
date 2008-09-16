#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <mechanism_control/MechanismState.h>

namespace TREX {

  class ArmStateAdapter: public ROSStateAdapter<mechanism_control::MechanismState> {
  public:
    ArmStateAdapter(const LabelStr& agentName, const TiXmlElement& configData, unsigned int _base)
      : ROSStateAdapter<mechanism_control::MechanismState> ( agentName, configData), base(_base) {
    }

    virtual ~ArmStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){

      /** Stored output from printing joints.
	  11 == gripper_roll_right_joint at -0
	  13 == wrist_flex_right_joint at 1.6342e-10
	  15 == forearm_roll_right_joint at 1.78346e-12
	  17 == elbow_flex_right_joint at 4.54146e-10
	  19 == upperarm_roll_right_joint at -4.41779e-12
	  21 == shoulder_pitch_right_joint at -2.39491e-10
	  23 == shoulder_pan_right_joint at -9.62199e-11

      for(unsigned int i=0;i<stateMsg.get_joint_states_size(); i++){
	std::cout << i << " == " << stateMsg.joint_states[i].name << " at " << stateMsg.joint_states[i].position << std::endl;
      }

      for(unsigned int i=0;i<stateMsg.get_actuator_states_size(); i++){
	std::cout << i << " == " << stateMsg.actuator_states[i].name << " at " << stateMsg.actuator_states[i].position << std::endl;
      }

      **/

      obs->push_back("shoulder_pan", new IntervalDomain(stateMsg.joint_states[base + 12].position));
      obs->push_back("shoulder_pitch", new IntervalDomain(stateMsg.joint_states[base + 10].position));
      obs->push_back("upperarm_roll", new IntervalDomain(stateMsg.joint_states[base + 8].position));
      obs->push_back("elbow_flex", new IntervalDomain(stateMsg.joint_states[base + 6].position));
      obs->push_back("forearm_roll", new IntervalDomain(stateMsg.joint_states[base + 4].position));
      obs->push_back("wrist_flex", new IntervalDomain(stateMsg.joint_states[base + 2].position));
      obs->push_back("gripper_roll", new IntervalDomain(stateMsg.joint_states[base + 0].position));    
    }

    const unsigned int base;
  };

  class RightArmStateAdapter: public ArmStateAdapter {
  public:
    RightArmStateAdapter(const LabelStr& agentName, const TiXmlElement& configData): ArmStateAdapter(agentName, configData, 11){}
  };

  class LeftArmStateAdapter: public  ArmStateAdapter {
  public:
    LeftArmStateAdapter(const LabelStr& agentName, const TiXmlElement& configData): ArmStateAdapter(agentName, configData, 12){}
  };

  // Allocate Factories
  TeleoReactor::ConcreteFactory<RightArmStateAdapter> r_ArmStateAdapter_Factory("RightArmStateAdapter");
  TeleoReactor::ConcreteFactory<LeftArmStateAdapter> l_ArmStateAdapter_Factory("LeftArmStateAdapter");
}

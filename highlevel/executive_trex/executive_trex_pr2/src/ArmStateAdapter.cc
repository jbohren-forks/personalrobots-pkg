#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <robot_msgs/MechanismState.h>
#include <vector>

namespace TREX {

  class ArmStateAdapter: public ROSStateAdapter<robot_msgs::MechanismState> {
  public:
    ArmStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<robot_msgs::MechanismState> ( agentName, configData) {
    }

    virtual ~ArmStateAdapter(){}

  private:

    bool readJointValue(const robot_msgs::MechanismState& mechanismStateMsg, const std::string& name, double& value){
      for(unsigned int i = 0; i < mechanismStateMsg.get_joint_states_size(); i++){
	const std::string& jointName = mechanismStateMsg.joint_states[i].name;
	if(name == jointName){
	  value = mechanismStateMsg.joint_states[i].position;
	  return true;
	}
      }

      return false;
    }

    void fillObservationParameters(ObservationByValue* obs){
      for(unsigned int i = 0; i<nddlNames().size(); i++){
	double value;
	if(readJointValue(stateMsg, rosNames()[i], value))
	   obs->push_back(nddlNames()[i], new IntervalDomain(value));
	else
	  ROS_ERROR("No joint %s.", (rosNames()[i]).c_str());
      }
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<ArmStateAdapter> ArmStateAdapter_Factory("ArmStateAdapter");
}

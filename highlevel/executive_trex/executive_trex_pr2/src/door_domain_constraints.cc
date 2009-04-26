#include <executive_trex_pr2/door_domain_constraints.hh>
#include <executive_trex_pr2/adapter_utilities.h>
#include <doors_core/executive_functions.h>

#include "Debug.hh"

using namespace TREX;

namespace executive_trex_pr2 {

  //*******************************************************************************************
  GetRobotPoseForDoorConstraint::GetRobotPoseForDoorConstraint(const LabelStr& name,
						 const LabelStr& propagatorName,
						 const ConstraintEngineId& constraintEngine,
						 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, makeScopeForToken(variables)),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _qx(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _qy(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _qz(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _qw(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _token_id(TREX::getParentToken(variables[7])),
     _range(static_cast<IntervalDomain&>(getCurrentDomain(variables[8]))){
  }

  std::vector<ConstrainedVariableId> GetRobotPoseForDoorConstraint::makeScopeForToken(const std::vector<ConstrainedVariableId>& variables){
    // If the scope is already inclusive of door message variables there is no need to expand it.
    if(variables.size() > 9)
      return variables;

    // Add all the parameters
    std::vector<ConstrainedVariableId> new_scope(variables);
    TokenId token = TREX::getParentToken(variables[3]);
    const std::vector<ConstrainedVariableId>& params = token->parameters();
    for(unsigned int i = 0; i< params.size(); i++)
      new_scope.push_back(params[i]);

    return new_scope;
  }

  /**
   * This constraint invokes a function from the door domain to compute a point normal to the door
   * @todo Integarte proper code from Wim. Also assumes good data
   */
  void GetRobotPoseForDoorConstraint::handleExecute(){
    debugMsg("trex:propagation:doors:get_robot_pose_for_door",  "BEFORE: " << toString());

    // Wait till all inputs are bound
    const std::vector<ConstrainedVariableId>& params = _token_id->parameters();
    for(unsigned int i = 0; i< params.size(); i++){
      ConstrainedVariableId param = params[i];
      if(!param->lastDomain().isSingleton()){
	debugMsg("trex:propagation:doors:get_robot_pose_for_door",  
		 "Skipping propagation since" << param->toString() << " is unbound.");
	return;
      }
    }

    robot_msgs::Door msg;
    AdapterUtilities::write(_token_id, msg);

    // Now make the calculation - How this works without knowledge of robot position is beyond me. Have to check with Wim.
    tf::Stamped<tf::Pose> tf_stamped_pose = getRobotPose(msg, _range.getSingletonValue());

    // Extract xy, and theta, which is yaw
    tf::Point position = tf_stamped_pose.getOrigin();
    tf::Quaternion quaternian = tf_stamped_pose.getRotation();

    debugMsg("trex:propagation:doors:get_robot_pose_for_door", "Results prior to intersection are: " << std::endl <<
	     " x=" << position.getX() << std::endl << 
	     " y=" << position.getY() << std::endl << 
	     " z=" << position.getZ() << std::endl << 
	     " qx=" << quaternian.getX() << std::endl << 
	     " qy=" << quaternian.getY() << std::endl << 
	     " qz=" << quaternian.getZ() << std::endl << 
	     " qw=" << quaternian.getW());

    _x.set(position.getX());
    _y.set(position.getY());
    _z.set(position.getZ());
    _qx.set(quaternian.getX());
    _qy.set(quaternian.getY());
    _qz.set(quaternian.getZ());
    _qw.set(quaternian.getW());

    debugMsg("trex:propagation:doors:get_robot_pose_for_door",  "AFTER: " << toString());
  } 
}

#include <executive_trex_pr2/door_domain_constraints.hh>
#include <doors_core/executive_functions.h>
#include <ROSAdapter.hh>

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
     _th(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _token_id(TREX::getParentToken(variables[3])),
     _range(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))){
    checkError(variables.size() == 5, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }

  std::vector<ConstrainedVariableId> GetRobotPoseForDoorConstraint::makeScopeForToken(const std::vector<ConstrainedVariableId>& variables){
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
      ConstrainedVariableId param = params[0];
      if(!param->lastDomain().isSingleton()){
	debugMsg("trex:propagation:doors:get_robot_pose_for_door",  
		 "Skipping propagation since" << param->toString() << " is unbound.");
	return;
      }
    }

    robot_msgs::Door msg;
    ROSAdapter::writeTokenToDoorMessage(_token_id, msg);

    // Now make the calculation - How this works without knowledge of robot position is beyond me. Have to check with Wim.
    tf::Stamped<tf::Pose> tf_stamped_pose = getRobotPose(msg, _range.getSingletonValue());

    // Extract xy, and theta, which is yaw
    double yaw,pitch,roll;
    btMatrix3x3 mat =  tf_stamped_pose.getBasis();
    mat.getEulerZYX(yaw, pitch, roll);
    _x.set(tf_stamped_pose.getOrigin().getX());
    _y.set(tf_stamped_pose.getOrigin().getY());
    _th.set(yaw);

    debugMsg("trex:propagation:doors:get_robot_pose_for_door",  "AFTER: " << toString());
  } 
}

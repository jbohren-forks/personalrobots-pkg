#include <executive_trex_pr2/door_domain_constraints.hh>
#include <doors_core/executive_functions.h>
#include "Debug.hh"

namespace executive_trex_pr2 {

  //*******************************************************************************************
  GetRobotPoseForDoorConstraint::GetRobotPoseForDoorConstraint(const LabelStr& name,
						 const LabelStr& propagatorName,
						 const ConstraintEngineId& constraintEngine,
						 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _th(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _range(static_cast<IntervalDomain&>(getCurrentDomain(variables[7]))){
    checkError(variables.size() == 8, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }
	 
  /**
   * This constraint invokes a function from the door domain to compute a point normal to the door
   * @todo Integarte proper code from Wim. Also assumes good data
   */
  void GetRobotPoseForDoorConstraint::handleExecute(){

    debugMsg("trex:propagation:doors:get_robot_pose_for_door",  "BEFORE: " << toString());

    // Wait till all inputs are bound
    if(!_x1.isSingleton() || !_y1.isSingleton() ||!_x2.isSingleton() || !_y2.isSingleton() || !_range.isSingleton())
      return;

    robot_msgs::Door msg;
    msg.frame_p1.x = _x1.getSingletonValue();
    msg.frame_p1.y = _y1.getSingletonValue();
    msg.frame_p2.x = _x2.getSingletonValue();
    msg.frame_p2.y = _y2.getSingletonValue();

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

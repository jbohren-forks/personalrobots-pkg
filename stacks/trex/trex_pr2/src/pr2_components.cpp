#include "TeleoReactor.hh"
#include "Assembly.hh"
#include "trex_ros/components.h"
#include <trex_pr2/topological_map.h>
#include <trex_pr2/door_domain_constraints.h>
#include <trex_pr2/master_reactor.h>
#include <tf/transform_listener.h>


namespace TREX {
  tf::TransformListener& getTransformListener(){
    static tf::TransformListener tf(ros::Duration(10));
    return tf;
  }
  
  class TFGetRobotPoseConstraint: public Constraint {
  public:
    TFGetRobotPoseConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables)
      : Constraint(name, propagatorName, constraintEngine, variables),
	_x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
	_y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
	_z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
	_qx(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
	_qy(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
	_qz(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
	_qw(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
	_frame_id(static_cast<StringDomain&>(getCurrentDomain(variables[7]))),
	_frame(LabelStr(_frame_id.getSingletonValue()).toString()),
	_propagated(false){
      checkError(_frame_id.isSingleton(), "The frame has not been specified for tf to get robot pose. See model for error." << _frame_id.toString());
      condDebugMsg(!_frame_id.isSingleton(), "trex:error:tf_get_robot_pose",  "Frame has not been specified" << variables[7]->toLongString());
    }
    
  private:
    virtual void handleExecute(){
      debugMsg("trex:debug:propagation:tf_get_robot_pose",  "BEFORE: " << TREX::timeString() << toString());
      
      // Obtain pose if not already called
      if(!_propagated){
	getPose(_pose);
      }
      
      if(_propagated){
	debugMsg("trex:debug:propagation:tf_get_robot_pose", "Compute pose <" <<
		 _pose.getOrigin().x() << ", " <<
		 _pose.getOrigin().y() << ", " <<
		 _pose.getOrigin().z() << ", " <<
		 _pose.getRotation().x() << ", " <<
		 _pose.getRotation().y() << ", " <<
		 _pose.getRotation().z() << ", " <<
		 _pose.getRotation().w() << ", >");
	
	getCurrentDomain(getScope()[0]).set(_pose.getOrigin().x());
	getCurrentDomain(getScope()[1]).set(_pose.getOrigin().y());
	getCurrentDomain(getScope()[2]).set(_pose.getOrigin().z());
	getCurrentDomain(getScope()[3]).set(_pose.getRotation().x());
	getCurrentDomain(getScope()[4]).set(_pose.getRotation().y());
	getCurrentDomain(getScope()[5]).set(_pose.getRotation().z());
	getCurrentDomain(getScope()[6]).set(_pose.getRotation().w());
      }
      debugMsg("trex:debug:propagation:tf_get_robot_pose",  "AFTER: " << TREX::timeString() << toString());
    }

    virtual void setSource(const ConstraintId& sourceConstraint) {
      const TFGetRobotPoseConstraint* source_ptr = (TFGetRobotPoseConstraint*) sourceConstraint;
      checkError(source_ptr != NULL, "Could not cast " << sourceConstraint->toLongString());
      _pose = source_ptr->_pose;
      _propagated = source_ptr->_propagated;
    }

    void getPose(tf::Stamped<tf::Pose>& pose){
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.setIdentity();
      robot_pose.frame_id_ = "base_footprint";

      try{
	debugMsg("trex:debug:timing:get_pose",  "A: " << TREX::timeString());
	tf::TransformListener& tfl = getTransformListener();
	debugMsg("trex:debug:timing:get_pose",  "B: " << TREX::timeString());
	robot_pose.stamp_ = ros::Time();
	tfl.canTransform(_frame, "base_footprint", robot_pose.stamp_, ros::Duration(3.0));
	tfl.transformPose(_frame, robot_pose, pose);
	_propagated = true;
	debugMsg("trex:debug:timing:get_pose",  "C: " << TREX::timeString());
      }
      catch(tf::LookupException& ex) {
	ROS_ERROR("No Transform available Error: %s\n", ex.what());
	return;
      }
      catch(tf::ConnectivityException& ex) {
	ROS_ERROR("Connectivity Error: %s\n", ex.what());
	return;
      }
      catch(tf::ExtrapolationException& ex) {
	ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      }
    }

    IntervalDomain& _x, _y, _z, _qx, _qy, _qz, _qw; // Pose is the output
    StringDomain& _frame_id;
    const std::string _frame;
    bool _propagated;
    tf::Stamped<tf::Pose> _pose;
  };

  class PlugsGetOffsetPose: public Constraint{
  public:
    PlugsGetOffsetPose(const LabelStr& name, const LabelStr& propagatorName,
                       const ConstraintEngineId& constraintEngine,
                       const std::vector<ConstrainedVariableId>& vars);
  private:
    void handleExecute();
    const AbstractDomain &x, &y, &z, &qx, &qy, &qz, &qw, &dist;
    AbstractDomain &x2, &y2, &z2, &qx2, &qy2, &qz2, &qw2;
  };

  PlugsGetOffsetPose::PlugsGetOffsetPose(const LabelStr& name,
                                         const LabelStr& propagatorName,
                                         const ConstraintEngineId& constraintEngine,
                                         const std::vector<ConstrainedVariableId>& vars)
    : Constraint(name, propagatorName, constraintEngine, vars),
      x(getCurrentDomain(vars[0])),
      y(getCurrentDomain(vars[1])),
      z(getCurrentDomain(vars[2])),
      qx(getCurrentDomain(vars[3])),
      qy(getCurrentDomain(vars[4])),
      qz(getCurrentDomain(vars[5])),
      qw(getCurrentDomain(vars[6])),
      dist(getCurrentDomain(vars[7])),
      x2(getCurrentDomain(vars[8])),
      y2(getCurrentDomain(vars[9])),
      z2(getCurrentDomain(vars[10])),
      qx2(getCurrentDomain(vars[11])),
      qy2(getCurrentDomain(vars[12])),
      qz2(getCurrentDomain(vars[13])),
      qw2(getCurrentDomain(vars[14]))
  {
    checkError(vars.size()==15, "Invalid arg count: " << vars.size());
  }

  void PlugsGetOffsetPose::handleExecute()
  {
    using geometry_msgs::Pose;
    debugMsg("trex:debug:propagation:world_model:plugs_get_offset_pose", "BEFORE: " << toString());
    if (x.isSingleton() && y.isSingleton() && z.isSingleton() && qx.isSingleton() &&
        qy.isSingleton() && qz.isSingleton() && qw.isSingleton() && dist.isSingleton()) {
      Pose outletPose;
      outletPose.position.x=x.getSingletonValue();
      outletPose.position.y=y.getSingletonValue();
      outletPose.position.z=z.getSingletonValue();
      outletPose.orientation.x=qx.getSingletonValue();
      outletPose.orientation.y=qy.getSingletonValue();
      outletPose.orientation.z=qz.getSingletonValue();
      outletPose.orientation.w=qw.getSingletonValue();


      // Determines the desired base position
      tf::Pose outlet_pose_tf;
      tf::poseMsgToTF(outletPose, outlet_pose_tf);
      tf::Pose desi_offset(tf::Quaternion(0,0,0), tf::Vector3(-dist.getSingletonValue(), 0.2, 0.0));
      tf::Pose target = outlet_pose_tf * desi_offset;
      Pose targetPose;
      tf::poseTFToMsg(target, targetPose);


      x2.set(targetPose.position.x);
      y2.set(targetPose.position.y);
      z2.set(targetPose.position.z);

      qx2.set(targetPose.orientation.x);
      qy2.set(targetPose.orientation.y);
      qz2.set(targetPose.orientation.z);
      qw2.set(targetPose.orientation.w);
    }
    debugMsg("trex:debug:propagation:world_model:plugs_get_offset_pose", "AFTER: " << toString());
  }





  class MapPoseMsgEqConstraint: public ParamEqConstraint {
  public:
    MapPoseMsgEqConstraint(const LabelStr& name,
			   const LabelStr& propagatorName,
			   const ConstraintEngineId& constraintEngine,
			   const std::vector<ConstrainedVariableId>& variables)
      : ParamEqConstraint(name, propagatorName, constraintEngine, variables,
			  "frame_id:x:y:z:qx:qy:qz:qw")
    {}
  };
 class PlugStowMsgEqConstraint: public ParamEqConstraint {
  public:
    PlugStowMsgEqConstraint(const LabelStr& name,
			const LabelStr& propagatorName,
			const ConstraintEngineId& constraintEngine,
			const std::vector<ConstrainedVariableId>& variables)
      : ParamEqConstraint(name, propagatorName, constraintEngine, variables,
			  "frame_id:time_stamp:stowed:x:y:z")
    {}
  };
  class DoorMsgEqConstraint: public ParamEqConstraint {
  public:
    DoorMsgEqConstraint(const LabelStr& name,
			const LabelStr& propagatorName,
			const ConstraintEngineId& constraintEngine,
			const std::vector<ConstrainedVariableId>& variables)
      : ParamEqConstraint(name, propagatorName, constraintEngine, variables,
			  "frame_id:time_stamp:latch_state:frame_p1_x:frame_p1_y:frame_p1_z:frame_p2_x:frame_p2_y:frame_p2_z:height:hinge:rot_dir:door_p1_x:door_p1_y:door_p1_z:door_p2_x:door_p2_y:door_p2_z:handle_x:handle_y:handle_z:travel_dir_x:travel_dir_y:travel_dir_z")
    {}
  };
}

namespace trex_pr2 {
  void registerComponents(bool playback, const Assembly& assembly){
    ROS_INFO("Registering trex_pr2 Schema");

    ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();

    REGISTER_CONSTRAINT(constraintEngine->getCESchema(), PlugsGetOffsetPose, "plugs_get_offset_pose", "Default");

    // Constraints for message binding
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::PlugStowMsgEqConstraint, "eq_plug_stow_msg", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::MapPoseMsgEqConstraint, "eq_map_pose_msg", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::DoorMsgEqConstraint, "eq_door_msg", "Default");

    // Register topological map constraints
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapInitializeFromFileConstraint, "map_initialize_from_file", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetNextMoveConstraint, "map_get_next_move", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapConnectorConstraint, "map_connector", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetRegionFromPositionConstraint, "map_get_region_from_position", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetDoorwayFromPointsConstraint, "map_get_doorway_from_points", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapIsDoorwayConstraint, "map_is_doorway", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetDoorStateConstraint, "map_get_door_state", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapNotifyDoorBlockedConstraint, "map_notify_door_blocked", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetNearestOutletConstraint, "map_get_nearest_outlet", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetOutletStateConstraint, "map_get_outlet_state", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetOutletApproachPoseConstraint, "map_get_outlet_approach_pose", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapNotifyOutletBlockedConstraint, "map_notify_outlet_blocked", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetNearestConnectorConstraint, "map_get_nearest_connector", "Default");
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::MapGetDoorApproachPoseConstraint, "map_get_door_approach_pose", "Default");

    // Register functions for calculations in the door domain
    REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
	trex_pr2::GetRobotPoseForDoorConstraint, "door_get_robot_pose_for_door", "Default");

    // Register SOLVER components for topological map.
    EUROPA::SOLVERS::ComponentFactoryMgr* cfm = (EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr");
    REGISTER_FLAW_FILTER(cfm, trex_pr2::MapConnectorFilter, MapConnectorFilter);
    REGISTER_FLAW_HANDLER(cfm, trex_pr2::MapConnectorSelector, MapConnectorSelector);
    REGISTER_FLAW_MANAGER(cfm, trex_pr2::TopologicalGoalManager, TopologicalGoalManager);
  }

  REGISTER_SCHEMA(&trex_pr2::registerComponents);

  void registerFactories(bool playback) {
    // Register special reactors
    new TREX::TeleoReactor::ConcreteFactory<trex_pr2::MasterReactor>("MasterReactor");
  }

  REGISTER_FACTORY(&trex_pr2::registerFactories);
}

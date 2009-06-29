#include "TeleoReactor.hh"
#include "Assembly.hh"
#include "trex_ros/components.h"
#include <trex_pr2/topological_map.h>
#include <trex_pr2/door_domain_constraints.h>
#include <trex_pr2/master_reactor.h>


namespace TREX {
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

  void registerPr2Components(bool playback, const Assembly& assembly){
    ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();
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

    // Register special reactors
    new TREX::TeleoReactor::ConcreteFactory<trex_pr2::MasterReactor>("MasterReactor");
  }
  REGISTER_SCHEMA(registerPr2Components);
}

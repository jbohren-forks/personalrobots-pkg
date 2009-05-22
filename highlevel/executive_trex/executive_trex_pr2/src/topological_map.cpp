/**
 * @author Conor McGann
 */
#include <executive_trex_pr2/topological_map.h>
#include <executive_trex_pr2/components.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "ConstrainedVariable.hh"
#include "Utilities.hh"
#include "Token.hh"
#include "Domains.hh"
#include "Utilities.hh"

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {


  //*******************************************************************************************
  MapGetNearestConnectorConstraint::MapGetNearestConnectorConstraint(const LabelStr& name,
							       const LabelStr& propagatorName,
							       const ConstraintEngineId& constraintEngine,
							       const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _connector(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))){
    checkError(variables.size() == 3, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetNearestConnectorConstraint::handleExecute(){
    // Wait till inputs are bound.
    if(!_x.isSingleton() || !_y.isSingleton())
      return;

    debugMsg("map:get_nearest_connector",  "BEFORE: "  << TREX::timeString() << toString());

    double x = _x.getSingletonValue();
    double y = _y.getSingletonValue();
    unsigned int connector_id = TopologicalMapAdapter::instance()->getNearestConnector(x, y);

    condDebugMsg(connector_id > 0, "trex:error", "No connector found for <" << x << ", " << y << ">");
    ROS_ASSERT(connector_id > 0); // If this is incorrect, then there is a bug in the topological map

    _connector.set(connector_id);

    debugMsg("map:get_nearest_connector",  "AFTER: "  << TREX::timeString() << toString());
  }

  //*******************************************************************************************
  MapGetDoorApproachPoseConstraint::MapGetDoorApproachPoseConstraint(const LabelStr& name,
								     const LabelStr& propagatorName,
								     const ConstraintEngineId& constraintEngine,
								     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _qx(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _qy(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _qz(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _qw(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _connector_id(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[7]))){
    checkError(variables.size() == 8, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetDoorApproachPoseConstraint::handleExecute(){

    // Wait till inputs are bound.
    if(!_connector_id.isSingleton())
      return;

    debugMsg("map:get_door_appoach_pose",  "BEFORE: "  << TREX::timeString() << toString());

    try{
      // Get the approach pose for that
      robot_msgs::Pose pose;
      TopologicalMapAdapter::instance()->getDoorApproachPose(_connector_id.getSingletonValue(), pose);
      _x.set(pose.position.x);
      _y.set(pose.position.y);
      _z.set(pose.position.z);
      _qx.set(pose.orientation.x);
      _qy.set(pose.orientation.y);
      _qz.set(pose.orientation.z);
      _qw.set(pose.orientation.w);
    }
    catch(...){
      debugMsg("trex:warning", "Invalid key value");
      _connector_id.empty();
    }

    debugMsg("map:get_door_approach_pose",  "AFTER: "  << TREX::timeString() << toString());
  }

  //*******************************************************************************************
  MapInitializeFromFileConstraint::MapInitializeFromFileConstraint(const LabelStr& name,
								   const LabelStr& propagatorName,
								   const ConstraintEngineId& constraintEngine,
								   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables), _map(NULL){
    checkError(variables.size() == 1, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    const StringDomain& dom = static_cast<const StringDomain&>(getCurrentDomain(variables[0]));
    checkError(dom.isSingleton(), dom.toString() << " is not a singleton when it should be.");
    const LabelStr fileName = dom.getSingletonValue();
    std::string fileNameStr = TREX::findFile(fileName.toString());
    std::ifstream is(fileNameStr.c_str());
    ROS_INFO("Loading the topological map from file");
    _map = new TopologicalMapAdapter(is);
  }

  MapInitializeFromFileConstraint::~MapInitializeFromFileConstraint(){
    delete _map;
  }

  //*******************************************************************************************
  MapNotifyDoorBlockedConstraint::MapNotifyDoorBlockedConstraint(const LabelStr& name,
								 const LabelStr& propagatorName,
								 const ConstraintEngineId& constraintEngine,
								 const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables), 
      _doorway(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))){
    checkError(variables.size() == 1, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }

  void MapNotifyDoorBlockedConstraint::handleExecute(){
    if(TopologicalMapAdapter::instance() == NULL){
      debugMsg("map", "No topological map present!");
      return;
    }

    if(_doorway.isSingleton()){
      debugMsg("map", "Notification that doorway [" << _doorway.getSingletonValue() << "] is blocked.");
      TopologicalMapAdapter::instance()->observeDoorBlocked(_doorway.getSingletonValue());
    }
  }

  //*******************************************************************************************
  MapNotifyOutletBlockedConstraint::MapNotifyOutletBlockedConstraint(const LabelStr& name,
								     const LabelStr& propagatorName,
								     const ConstraintEngineId& constraintEngine,
								     const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables), 
      _outlet(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))){
    checkError(variables.size() == 1, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }

  void MapNotifyOutletBlockedConstraint::handleExecute(){
    if(_outlet.isSingleton()){
      debugMsg("map", "Notification that outlet [" << _outlet.getSingletonValue() << "] is blocked.");
      TopologicalMapAdapter::instance()->observeOutletBlocked(_outlet.getSingletonValue());
    }
  }

  //*******************************************************************************************
  MapGetNearestOutletConstraint::MapGetNearestOutletConstraint(const LabelStr& name,
							       const LabelStr& propagatorName,
							       const ConstraintEngineId& constraintEngine,
							       const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _outlet(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))){
    checkError(variables.size() == 3, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetNearestOutletConstraint::handleExecute(){
    // Wait till inputs are bound.
    if(!_x.isSingleton() || !_y.isSingleton())
      return;

    debugMsg("map:get_nearest_outlet",  "BEFORE: "  << TREX::timeString() << toString());

    double x = _x.getSingletonValue();
    double y = _y.getSingletonValue();
    unsigned int outlet_id = TopologicalMapAdapter::instance()->getNearestOutlet(x, y);

    checkError(outlet_id > 0, "No outlet found for <" << x << ", " << y << ">");

    _outlet.set(outlet_id);

    debugMsg("map:get_nearest_outlet",  "AFTER: "  << TREX::timeString() << toString());
  }

  //*******************************************************************************************
  MapGetOutletApproachPoseConstraint::MapGetOutletApproachPoseConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _qx(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _qy(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _qz(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _qw(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _outlet_id(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[7]))){
    checkError(variables.size() == 8, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetOutletApproachPoseConstraint::handleExecute(){

    // Wait till inputs are bound.
    if(!_outlet_id.isSingleton())
      return;

    // If outlet id is 0 then skip.
    if(_outlet_id.getSingletonValue() == 0){
      debugMsg("map:get_outlet_appoach_pose", "Ignoring outlet id since it is a NO_KEY");
      return;
    }

    debugMsg("map:get_outlet_appoach_pose",  "BEFORE: "  << TREX::timeString() << toString());

    try{
      robot_msgs::Pose pose;
      TopologicalMapAdapter::instance()->getOutletApproachPose(_outlet_id.getSingletonValue(), pose);
      _x.set(pose.position.x);
      _y.set(pose.position.y);
      _z.set(pose.position.z);
      _qx.set(pose.orientation.x);
      _qy.set(pose.orientation.y);
      _qz.set(pose.orientation.z);
      _qw.set(pose.orientation.w);
    }
    catch(...){
      debugMsg("trex:warning", "Invalid outlet id (" << _outlet_id.getSingletonValue() << ")");
      _outlet_id.empty();
    }

    debugMsg("map:get_outlet_approach_pose",  "AFTER: "  << TREX::timeString() << toString());
  }

  //*******************************************************************************************
  MapGetOutletStateConstraint::MapGetOutletStateConstraint(const LabelStr& name,
							    const LabelStr& propagatorName,
							    const ConstraintEngineId& constraintEngine,
							    const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _qx(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _qy(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _qz(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _qw(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _outlet(static_cast<const IntervalIntDomain&>(getCurrentDomain(variables[7]))){
    checkError(variables.size() == 8, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetOutletStateConstraint::handleExecute(){

    // Wait till inputs are bound.
    if(!_outlet.isSingleton())
      return;

    debugMsg("map:get_outlet_state",  "BEFORE: "  << TREX::timeString() << toString());
    robot_msgs::Pose outlet_pose;
    TopologicalMapAdapter::instance()->getOutletState(_outlet.getSingletonValue(), outlet_pose);
    _x.set(outlet_pose.position.x);
    _y.set(outlet_pose.position.y);
    _z.set(outlet_pose.position.z);
    _qx.set(outlet_pose.orientation.x);
    _qy.set(outlet_pose.orientation.y);
    _qz.set(outlet_pose.orientation.z);
    _qw.set(outlet_pose.orientation.w);

    debugMsg("map:get_outlet_state",  "AFTER: "  << TREX::timeString() << toString());
  }



  //*******************************************************************************************
  MapConnectorConstraint::MapConnectorConstraint(const LabelStr& name,
						 const LabelStr& propagatorName,
						 const ConstraintEngineId& constraintEngine,
						 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _connector(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))){
    checkError(variables.size() == 3, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * @brief When the connector id is bound, we bind the values for x, y, and th based on a lookup in the topological map.
   * We can also bind the value for the connector id based on x and y inputs. This enforces the full relation.
   */
  void MapConnectorConstraint::handleExecute(){
    debugMsg("map:connector",  "BEFORE: "  << TREX::timeString() << toString());
    if(_connector.isSingleton()){
      unsigned int connector_id = (unsigned int) _connector.getSingletonValue();
      double x, y;

      // If we have a pose, bind the variables
      if(TopologicalMapAdapter::instance()->getConnectorPosition(connector_id, x, y)){
	_x.set(x);

	if(!_x.isEmpty()) 
	  _y.set(y);
      }
    }

    // If x and y are set, we can bind the connector.
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int connector_id = TopologicalMapAdapter::instance()->getConnector(_x.getSingletonValue(), _y.getSingletonValue());
      _connector.set(connector_id);
    }
    debugMsg("map:get_connector",  "AFTER: " << TREX::timeString()  << toString());
  }

  //*******************************************************************************************
  MapGetRegionFromPositionConstraint::MapGetRegionFromPositionConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _region(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))){
    checkError(variables.size() == 3, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetRegionFromPositionConstraint::handleExecute(){
    if(!_x.isSingleton() || !_y.isSingleton())
      return;

    debugMsg("map:get_region_from_position",  "BEFORE: "  << TREX::timeString() << toString());

    double x = _x.getSingletonValue();
    double y = _y.getSingletonValue();

    unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(x, y);
    _region.set(region_id);

    debugMsg("map:get_region_from_position",  "AFTER: " << TREX::timeString()  << toString());

    condDebugMsg(_region.isEmpty(), "map:get_region_from_position", 
		 "isObstacle(" << x << ", " << y << ") == " << (TopologicalMapAdapter::instance()->isObstacle(x, y) ? "TRUE" : "FALSE"));
  } 

  //*******************************************************************************************
  MapGetDoorwayFromPointsConstraint::MapGetDoorwayFromPointsConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _region(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))){
    checkError(variables.size() == 5, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetDoorwayFromPointsConstraint::handleExecute(){
    // Wait till inputs are bound. This is a common pattern for functions and could be mapped into a base class with an input and output
    // list
    if(!_x1.isSingleton() || !_y1.isSingleton() || !_x2.isSingleton() || !_y2.isSingleton())
      return;

    debugMsg("map:get_doorway_from_points",  "BEFORE: "  << TREX::timeString() << toString());

    unsigned int region_id = TopologicalMapAdapter::instance()->getNearestDoorway(_x2.getSingletonValue(), _y2.getSingletonValue());

    // If it is not a doorway, then set to zero
    getCurrentDomain(getScope()[0]).set(region_id);

    debugMsg("map:get_doorway_from_points",  "AFTER: "  << TREX::timeString() << toString());
  }
    
  //*******************************************************************************************
  MapIsDoorwayConstraint::MapIsDoorwayConstraint(const LabelStr& name,
						 const LabelStr& propagatorName,
						 const ConstraintEngineId& constraintEngine,
						 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _result(static_cast<BoolDomain&>(getCurrentDomain(variables[0]))),
     _region(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[1]))){
    checkError(variables.size() == 2, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  void MapIsDoorwayConstraint::handleExecute(){
    // If inputs are not bound, return
    if(!_region.isSingleton())
      return;

    debugMsg("map:is_doorway",  "BEFORE: " << TREX::timeString()  << toString());

    unsigned int region_id = _region.getSingletonValue();
    bool is_doorway(true);

    TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway);
    _result.set(is_doorway);

    debugMsg("map:is_doorway",  "AFTER: " << TREX::timeString()  << toString());
  }

  //*******************************************************************************************
  MapGetDoorStateConstraint::MapGetDoorStateConstraint(const LabelStr& name,
						       const LabelStr& propagatorName,
						       const ConstraintEngineId& constraintEngine,
						       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables),
      _token_id(TREX::getParentToken(variables[0])),
      _door_id(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[1]))){
    checkError(variables.size() == 2, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }

  void MapGetDoorStateConstraint::handleExecute(){
    // If the door id is not a singleton then there is nothing to do
    if(!_door_id.isSingleton())
      return;

    debugMsg("map:get_door_state",  "BEFORE: " << TREX::timeString()  << toString());

    unsigned int id = (unsigned int)_door_id.getSingletonValue();

    door_msgs::Door door_state;
    if(!TopologicalMapAdapter::instance()->getDoorState(id, door_state)){
      debugMsg("map:get_door_state",  "No door message available for " << id);
      _door_id.empty();
    }
    else {
      static const LabelStr MAP_FRAME("map");
      if(!apply(MAP_FRAME, "frame_id") ||
	 !apply(door_state.frame_p1.x, "frame_p1_x") ||
	 !apply(door_state.frame_p1.y, "frame_p1_y") ||
	 !apply(door_state.frame_p1.z, "frame_p1_z") ||
	 !apply(door_state.frame_p2.x, "frame_p2_x") ||
	 !apply(door_state.frame_p2.y, "frame_p2_y") ||
	 !apply(door_state.frame_p2.z, "frame_p2_z") ||
	 !apply(door_state.height, "height") ||
	 !apply(door_state.hinge, "hinge") ||
	 !apply(door_state.rot_dir, "rot_dir") ||
	 !apply(door_state.door_p1.x, "door_p1_x") ||
	 !apply(door_state.door_p1.y, "door_p1_y") ||
	 !apply(door_state.door_p1.z, "door_p1_z") ||
	 !apply(door_state.door_p2.x, "door_p2_x") ||
	 !apply(door_state.door_p2.y, "door_p2_y") ||
	 !apply(door_state.door_p2.z, "door_p2_z") ||
	 !apply(door_state.handle.x, "handle_x") ||
	 !apply(door_state.handle.y, "handle_y") ||
	 !apply(door_state.handle.z, "handle_z")||
	 !apply(door_state.latch_state, "latch_state")) {
      }
    }
      
    debugMsg("map:get_door_state",  "AFTER: " << TREX::timeString()  << toString());
  }
  
  std::string MapGetDoorStateConstraint::toString() const {
    std::stringstream ss;
    ss << Constraint::toString() << std::endl;

    for(std::vector<ConstrainedVariableId>::const_iterator it = _token_id->parameters().begin(); it != _token_id->parameters().end(); ++it){
      ConstrainedVariableId var = *it;
      ss << var->getName().toString() << "(" << var->getKey() << ") == " << var->lastDomain().toString() << std::endl;
    }

    return ss.str();
  }

  /**
   * Reads a numeric value into a token parameter by name
   */
  bool MapGetDoorStateConstraint::apply(double value, const char* param_name){
    ConstrainedVariableId var = _token_id->getVariable(param_name);
    ROS_ASSERT(var.isValid());

    AbstractDomain& dom = getCurrentDomain(var);
    condDebugMsg(dom.isNumeric(), "map:get_door_state", "Setting " << value  << " for " << param_name << " in " << dom.toString());
    condDebugMsg(!dom.isNumeric(), "map:get_door_state", "Setting " << LabelStr(value).toString() << " for " << param_name << " in " << dom.toString());
    dom.set(value);
    return !dom.isEmpty();
  }

  //*******************************************************************************************
  MapConnectorFilter::MapConnectorFilter(const TiXmlElement& configData)
    : SOLVERS::FlawFilter(configData, true),
      _source_x(extractData(configData, "source_x")),
      _source_y(extractData(configData, "source_y")),
      _final_x(extractData(configData, "final_x")),
      _final_y(extractData(configData, "final_y")),
      _target_connector(extractData(configData, "target_connector")){}

  /**
   * The filter will exclude the entity if it is not the target variable of a token with the right structure
   * with all inputs bound so that the selection can be made based on sufficient data
   */
  bool MapConnectorFilter::test(const EntityId& entity){
    // It should be a variable. So if it is not, filter it out
    if(!ConstrainedVariableId::convertable(entity))
      return true;

    // Its name should match the target, it should be enumerated, and it should not be a singleton.
    ConstrainedVariableId var = (ConstrainedVariableId) entity;
    if(var->getName() != _target_connector || !var->lastDomain().isNumeric()){
      debugMsg("MapConnectorFilter:test", "Excluding " << var->getName().toString());
      return true;
    }

    // It should have a parent that is a token
    if(var->parent().isNoId() || !TokenId::convertable(var->parent()))
      return true;
    
    TokenId parent = (TokenId) var->parent();
    ConstrainedVariableId source_x = parent->getVariable(_source_x, false);
    ConstrainedVariableId source_y = parent->getVariable(_source_y, false);
    ConstrainedVariableId final_x = parent->getVariable(_final_x, false);
    ConstrainedVariableId final_y = parent->getVariable(_final_y, false);
  
    return(noGoodInput(parent, _source_x) ||
	   noGoodInput(parent, _source_y) ||
	   noGoodInput(parent, _final_x) ||
	   noGoodInput(parent, _final_y));
  }

  bool MapConnectorFilter::noGoodInput(const TokenId& parent_token, const LabelStr& var_name) const {
    ConstrainedVariableId var = parent_token->getVariable(var_name, false);
    bool result = true;
    if(var.isNoId()){
      debugMsg("MapConnectorFilter:test", "Excluding because of invalid source:" << var_name.toString());
    }
    else if(!var->lastDomain().isSingleton()){
      debugMsg("MapConnectorFilter:test", 
	       "Excluding because of unbound inputs on:" << 
	       var_name.toString() << "[" << var->getKey() << "] with " << var->lastDomain().toString());
    }
    else{
      result = false;
    }

    return result;
  }

  //*******************************************************************************************
  MapConnectorSelector::MapConnectorSelector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation)
    : SOLVERS::UnboundVariableDecisionPoint(client, flawedVariable, configData, explanation),
      _source_x(extractData(configData, "source_x")),
      _source_y(extractData(configData, "source_y")),
      _final_x(extractData(configData, "final_x")),
      _final_y(extractData(configData, "final_y")),
      _target_connector(extractData(configData, "target_connector")){

    // Verify expected structure for the variable - wrt other token parameters
    checkError(flawedVariable->parent().isId() && TokenId::convertable(flawedVariable->parent()), 
	       "The variable configuration is incorrect. The Map_connector_filter must not be working. Expect a token parameter with source connector and final region.");


    // Obtain source and final 
    TokenId parent = (TokenId) flawedVariable->parent();
    ConstrainedVariableId source_x = parent->getVariable(_source_x, false);
    ConstrainedVariableId source_y = parent->getVariable(_source_y, false);
    ConstrainedVariableId final_x = parent->getVariable(_final_x, false);
    ConstrainedVariableId final_y = parent->getVariable(_final_y, false);

    // Double check source variables
    checkError(source_x.isId(), "Bad filter. Invalid source name: " << _source_x.toString() << " on token " << parent->toString());
    checkError(source_y.isId(), "Bad filter. Invalid source name: " << _source_y.toString() << " on token " << parent->toString());
    checkError(source_x->lastDomain().isSingleton(), "Bad Filter. Source is not a singleton: " << source_x->toString());
    checkError(source_y->lastDomain().isSingleton(), "Bad Filter. Source is not a singleton: " << source_y->toString());

    // Double check destination variables
    checkError(final_x.isId(), "Bad filter. Invalid final name: " << _final_x.toString() << " on token " << parent->toString());
    checkError(final_y.isId(), "Bad filter. Invalid final name: " << _final_y.toString() << " on token " << parent->toString());
    checkError(final_x->lastDomain().isSingleton(), "Bad Filter. Final is not a singleton: " << final_x->toString());
    checkError(final_y->lastDomain().isSingleton(), "Bad Filter. Final is not a singleton: " << final_y->toString());

    // Compose all the choices. 
    // Given a connector, and a target, generate the choices for all connectors
    // adjacent to this connector, and order them to minimize cost.
    double x0, y0, x1, y1;
    x0 = source_x->lastDomain().getSingletonValue();
    y0 = source_y->lastDomain().getSingletonValue();
    x1 = final_x->lastDomain().getSingletonValue();
    y1 = final_y->lastDomain().getSingletonValue();

    TopologicalMapAdapter::instance()->getLocalConnectionsForGoal(_sorted_choices, x0, y0, x1, y1);

    _sorted_choices.sort();
    _choice_iterator = _sorted_choices.begin();

    debugMsg("map", "Evaluating from <" << x0 << ", " << y0 << "> to <" << x1 << ", " << y1 << ">. " << toString());
  }

  std::string MapConnectorSelector::toString() const {
    std::stringstream ss;
    ss << "Choices are: " << std::endl;
    for(std::list<ConnectionCostPair>::const_iterator it = _sorted_choices.begin(); it != _sorted_choices.end(); ++it){
      const ConnectionCostPair& choice = *it;
      double x, y;
      TopologicalMapAdapter::instance()->getConnectorPosition(choice.id, x, y);

      if(choice.id == 0)
	ss << "<0, GOAL, " << choice.cost << ">" << std::endl;
      else
	ss << "<" << choice.id << ", (" << x << ", " << y << ") " << choice.cost << ">" << std::endl;
    }

    return ss.str();
  }

  bool MapConnectorSelector::hasNext() const { return _choice_iterator != _sorted_choices.end(); }

  double MapConnectorSelector::getNext(){
    ConnectionCostPair c = *_choice_iterator;
    debugMsg("MapConnectorSelector::getNext", "Selecting " << c.id << std::endl);
    ++_choice_iterator;
    return c.id;
  }

  /************************************************************************
   * Map Adapter implementation
   ************************************************************************/

  TopologicalMapAdapter* TopologicalMapAdapter::_singleton = NULL;

  TopologicalMapAdapter* TopologicalMapAdapter::instance(){
    return _singleton;
  }

  TopologicalMapAdapter::TopologicalMapAdapter(std::istream& in){

    if(_singleton != NULL)
      delete _singleton;

    _singleton = this;

    // Temporary fix due to mismatch between 5cm map and 2.5cm map
    _map = topological_map::TopologicalMapPtr(new topological_map::TopologicalMap(in, 1.0, 1e9, 1e9));

    debugMsg("map:initialization", toPPM());
  }

  std::string TopologicalMapAdapter::toPPM(){
    std::ofstream of("topological_map.ppm");
    _map->writePpm(of);

    return "Output map in local directory: topological_map.ppm";
  }

  TopologicalMapAdapter::TopologicalMapAdapter(const topological_map::OccupancyGrid& grid, double resolution) {

    if(_singleton != NULL)
      delete _singleton;

    _singleton = this;

    _map = topological_map::topologicalMapFromGrid(grid, resolution, 2, 1, 1, 0, "local");

    toPostScriptFile();
  }

  TopologicalMapAdapter::~TopologicalMapAdapter(){
    _singleton = NULL;
  }

  unsigned int TopologicalMapAdapter::getNearestDoorway(double x, double y){
    debugMsg("trex:timing:getNearestDoorway", "BEFORE:"  << TREX::timeString());
    const topological_map::RegionIdSet& all_regions = _map->allRegions();
    double distance = PLUS_INFINITY;
    unsigned int nearest_doorway = 0;
    for(topological_map::RegionIdSet::const_iterator it = all_regions.begin(); it != all_regions.end(); ++it){
      bool is_doorway(false);
      topological_map::RegionId region_id = *it;
      isDoorway(region_id, is_doorway);
      if(is_doorway){
	door_msgs::Door door_msg;
	getDoorState(region_id, door_msg);
	double frame_center_x = (door_msg.frame_p1.x + door_msg.frame_p2.x) / 2.0;
	double frame_center_y = (door_msg.frame_p1.y + door_msg.frame_p2.y) / 2.0;
	double distance_to_frame = sqrt(pow(x - frame_center_x, 2) + pow(y - frame_center_y, 2));
	if(distance_to_frame < distance){
	  distance = distance_to_frame;
	  nearest_doorway = region_id;
	}
      }
    }

    debugMsg("trex:timing:getNearestDoorway", "AFTER:"  << TREX::timeString());

    debugMsg("map:getNearestDoorway", 
	     "Found doorway " << nearest_doorway << " within " << distance << " meters from <" << x << ", " << y << ">");

    return nearest_doorway;
  }

  unsigned int TopologicalMapAdapter::getRegion(double x, double y){
    unsigned int result = 0;
    try{
      result = _map->containingRegion(topological_map::Point2D(x, y));
    }
    catch(...){}
    return result;
  }

  topological_map::RegionPtr TopologicalMapAdapter::getRegionCells(unsigned int region_id){
    return _map->regionCells(region_id);
  }

  unsigned int TopologicalMapAdapter::getConnector(double x, double y){
    unsigned int result = 0;
    try{
      result = _map->pointConnector(topological_map::Point2D(x, y));
    }
    catch(...){}
    return result;
  }

  bool TopologicalMapAdapter::getConnectorPosition(unsigned int connector_id, double& x, double& y){
    try{
      topological_map::Point2D p = _map->connectorPosition(connector_id);
      x = p.x;
      y = p.y;
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::getRegionConnectors(unsigned int region_id, std::vector<unsigned int>& connectors){
    try{
      connectors = _map->adjacentConnectors(region_id);
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::getConnectorRegions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b){
    try{
      topological_map::RegionPair pair = _map->adjacentRegions(connector_id);
      region_a = pair.first;
      region_b = pair.second;
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::getDoorState(unsigned int doorway_id, door_msgs::Door& door_state){
    try{
      // Check if it is a valid doorway id
      bool is_doorway(false);
      if(!isDoorway(doorway_id, is_doorway) || !is_doorway)
	return false;

      // Fill the door message and return
      door_state =_map->regionDoor(doorway_id);

      return true;
    }
    catch(...){}
    return false;
  }


  bool TopologicalMapAdapter::isDoorway(unsigned int region_id, bool& result){
    try{
      int region_type = _map->regionType(region_id);
      result = (region_type ==  topological_map::DOORWAY);
      return true;
    }
    catch(...){}
    result = false;
    return false;
  }

  bool TopologicalMapAdapter::isObstacle(double x, double y) {
    return _map->isObstacle(topological_map::Point2D(x, y));
  }


  double TopologicalMapAdapter::cost(double from_x, double from_y, double to_x, double to_y){

    // Obtain the distance point to point
    return sqrt(pow(from_x - to_x, 2) + pow(from_y - to_y, 2));

    /*
    std::pair<bool, double> result = _map->distanceBetween(topological_map::Point2D(from_x, from_y), topological_map::Point2D(to_x, to_y));

    // If there is no path then return infinint
    if(!result.first)
      return PLUS_INFINITY;


    // Otherwise we return the euclidean distance between source and target
    return result.second;
    */
  }

  double TopologicalMapAdapter::cost(double from_x, double from_y, unsigned int connector_id){
    double x(0), y(0);

    // If there is no connector for this id then we have an infinite cost
    if(!getConnectorPosition(connector_id, x, y))
      return PLUS_INFINITY;

    return cost(from_x, from_y, x, y);
  }

  void TopologicalMapAdapter::getConnectorCosts(double x0, double y0, double x1, double y1, std::vector< std::pair<topological_map::ConnectorId, double> >& results){
    const topological_map::Point2D source_point(x0, y0);
    const topological_map::Point2D target_point(x1, y1);
    try{
      results = _map->connectorCosts(source_point, target_point);
    }
    catch(std::runtime_error e){
      ROS_ERROR(e.what()); 
    }
    catch(...){
      debugMsg("map:getConnectorCosts", "failed for <" << x0 << ", " << y0 << "> => <" << x1 << ", " << y1 << ">"); 
    }
  }

  void TopologicalMapAdapter::getLocalConnectionsForGoal(std::list<ConnectionCostPair>& results, double x0, double y0, double x1, double y1){
    unsigned int this_region =  TopologicalMapAdapter::instance()->getRegion(x0, y0);
    condDebugMsg(this_region == 0, "map", "No region for <" << x0 << ", " << y0 <<">");
    unsigned int final_region =  TopologicalMapAdapter::instance()->getRegion(x1, y1);
    condDebugMsg(final_region == 0, "map", "No region for <" << x1 << ", " << y1 <<">");

    // If the source point and final point can be connected then we consider an option of going
    // directly to the point rather than thru a connector. To figure this out we should look the region at
    // the source connector and see if it is a connector for the final region
    if(this_region == final_region)
      results.push_back(ConnectionCostPair(0, cost(x0, y0, x1, y1)));

    const topological_map::Point2D source_point(x0, y0);
    const topological_map::Point2D target_point(x1, y1);
    std::vector< std::pair<topological_map::ConnectorId, double> > connector_cost_pairs = _map->connectorCosts(source_point, target_point);
    for(std::vector< std::pair<topological_map::ConnectorId, double> >::const_iterator it = connector_cost_pairs.begin();
	it != connector_cost_pairs.end(); ++it){

      results.push_back(ConnectionCostPair(it->first, it->second));
    }

    /*
    // Now iterate over the connectors in this region and compute the heuristic cost estimate. We exclude the source connector
    // since we have just arrived here
    std::vector<unsigned int> final_region_connectors;
    getRegionConnectors(final_region, final_region_connectors);
    unsigned int source_connector = getConnector(x0, y0);
    std::vector<unsigned int> connectors;
    getRegionConnectors(this_region, connectors);
    for(std::vector<unsigned int>::const_iterator it = connectors.begin(); it != connectors.end(); ++it){
      unsigned int connector_id = *it;
      if(connector_id != source_connector){
	results.push_back(ConnectionCostPair(connector_id, cost(x0, y0, connector_id) + cost(x1, y1, connector_id)));
      }
    }
    */
  }

  void TopologicalMapAdapter::toPostScriptFile(){
    std::ofstream of("topological_map.ps");

    // Print header
    of << "%%!PS\n";
    of << "%%%%Creator: Conor McGann (Willow Garage)\n";
    of << "%%%%EndComments\n";

    of << "2 setlinewidth\n";
    of << "newpath\n";

    std::vector<unsigned int> regions;
    std::vector<unsigned int> connectors;
    unsigned int region_id = 1;
    while(getRegionConnectors(region_id, connectors)){

      // Set color based on region type
      bool is_doorway(true);
      isDoorway(region_id, is_doorway);
      if(is_doorway){
	of << "1\t0\t0\tsetrgbcolor\n";
      }
      else
	of << "0\t1\t0\tsetrgbcolor\n";

      for(unsigned int i = 0; i < connectors.size(); i++){
	double x_i, y_i;
	getConnectorPosition(connectors[i], x_i, y_i);
	for(unsigned int j = 0; j < connectors.size(); j++){
	  double x_j, y_j;
	  getConnectorPosition(connectors[j], x_j, y_j);
	  if(i != j){
	    of << x_i * 10 << "\t" << y_i * 10 << "\tmoveto\n";
	    of << x_j * 10 << "\t" << y_j * 10<< "\tlineto\n";
	  }
	}
      }

      of << "closepath stroke\n";

      region_id++;
    }

    // Footer
    of << "showpage\n%%%%EOF\n";
    of.close();
  }

  void TopologicalMapAdapter::observeDoorBlocked(unsigned int door_id){
    _map->observeDoorTraversal(door_id, false, ros::Time::now());
  }

  unsigned int TopologicalMapAdapter::getNearestOutlet(double x, double y){
    topological_map::Point2D p(x, y);
    return _map->nearestOutlet(p);
  }

  unsigned int TopologicalMapAdapter::getNearestConnector(double x, double y){
    unsigned int connector_id(0);
    double distance(PLUS_INFINITY);
    unsigned int region_id = getRegion(x, y);
    std::vector<unsigned int> connectors;
    getRegionConnectors(region_id, connectors);
    for(std::vector<unsigned int>::const_iterator it = connectors.begin(); it != connectors.end(); ++it){
      unsigned int candidate = *it;
      double cx, cy;
      TopologicalMapAdapter::instance()->getConnectorPosition(candidate, cx, cy);
      double distance_to_connector = sqrt(pow(x-cx, 2) + pow(y-cy,2));
      if(distance_to_connector < distance){
	distance = distance_to_connector;
	connector_id = candidate;
      }
    }

    debugMsg("map:getNearestConnector", 
	     "Selecting connector " << connector_id << " in region " << region_id << " a distance " << distance << " from <" << x << ", " << y << ">");

    return connector_id;
  }

  void TopologicalMapAdapter::getOutletState(unsigned int outlet_id, robot_msgs::Pose& outlet_pose){
    topological_map::OutletInfo outlet_info = _map->outletInfo(outlet_id);
    outlet_pose.position.x = outlet_info.x;
    outlet_pose.position.y = outlet_info.y;
    outlet_pose.position.z = outlet_info.z;
    outlet_pose.orientation.x = outlet_info.qx;
    outlet_pose.orientation.y = outlet_info.qy;
    outlet_pose.orientation.z = outlet_info.qz;
    outlet_pose.orientation.w = outlet_info.qw;
  }

  /**
   * Impementation uses a distance of 1 meter and an error bound of 1 meter. Assume that the outlet pose can subsequently
   * be used.
   */
  void TopologicalMapAdapter::getOutletApproachPose(unsigned int outlet_id, robot_msgs::Pose& approach_pose){
    topological_map::OutletInfo outlet_info = _map->outletInfo(outlet_id);
    topological_map::Point2D p = _map->outletApproachPosition (outlet_id, 1.0, 0.3);
    approach_pose.position.x = p.x;
    approach_pose.position.y = p.y;
    approach_pose.position.z = 0;

    double dx = outlet_info.x - approach_pose.position.x;
    double dy = outlet_info.y - approach_pose.position.y;
    double heading = atan2(dy, dx);
    tf::QuaternionTFToMsg (tf::Quaternion(heading, 0.0, 0.0), approach_pose.orientation);
  }

  /**
   * Impementation uses a distance of 1 meter
   */
  void TopologicalMapAdapter::getDoorApproachPose(unsigned int connector_id, robot_msgs::Pose& approach_pose){
    topological_map::Point2D p = _map->doorApproachPosition (connector_id, 1.0);
    approach_pose.position.x = p.x;
    approach_pose.position.y = p.y;
    approach_pose.position.z = 0;


    // Orientation to face the connector
    robot_msgs::Vector3 orientation;
    double x, y;
    TopologicalMapAdapter::instance()->getConnectorPosition(connector_id, x, y);
    orientation.x = x - p.x;
    orientation.y = y - p.y;
    orientation.z = 0;
    tf::QuaternionTFToMsg(tf::Quaternion(atan2(orientation.y,orientation.x), 0.0, 0.0), approach_pose.orientation);
  }

  void TopologicalMapAdapter::observeOutletBlocked(unsigned int outlet_id){
    _map->observeOutletBlocked(outlet_id);
  }

  /**
   * Algorithm to see if we are crossing a doorway
   * 1. We have the target connector. We know this is really close - in fact it is within the grid resolution of being in the doorway
   * 2. We have the point where we are.
   *
   * Find the point a small distance (0.1 m) from the target connector in the direction of the current point. This is important since the current point may be pretty far away
   * but the target connector is really on the doorway border.
   */
  bool TopologicalMapAdapter::isDoorway(double current_x, double current_y, unsigned int doorway_connector){

    // Regions for next conecttor are a and b
    unsigned int region_a, region_b;
    getConnectorRegions(doorway_connector, region_a, region_b);

    // There can be many connectors for the current region
    unsigned int current_region = getRegion(current_x, current_y);
    std::vector<unsigned int> current_region_connectors;
    getRegionConnectors(current_region, current_region_connectors);

    // Iterate over the connectors.
    unsigned int shared_region(0);
    for(std::vector<unsigned int>::const_iterator it = current_region_connectors.begin(); it != current_region_connectors.end(); ++it){
      unsigned int a, b;
      getConnectorRegions(*it, a, b);
      if (a == region_a || a == region_b){
	shared_region = a;
	break;
      }
      else if(b == region_a || b == region_b){
	shared_region = b;
	break;
      }
    }
    bool is_doorway(false);

    if(shared_region > 0){
      debugMsg("map:get_next_move", "Found shared region " << shared_region);
      isDoorway(shared_region, is_doorway);

      debugMsg("map:get_next_move", shared_region << (is_doorway ? " is " : " is not ") << "a doorway");
    }

    /*
    double x, y;
    getConnectorPosition(doorway_connector, x, y);
    KDL::Vector current(current_x, current_y, 0);
    KDL::Vector connector(x, y, 0);
    KDL::Vector delta = current - connector;
    delta.Normalize();
    KDL::Vector region_point = connector + (delta * 0.2);
    unsigned int region_id = getRegion(region_point.x(), region_point.y());

    debugMsg("map:get_next_move", "Checking point " << region_point << " in region " << region_id);
    debugMsg("map:get_next_move", "Delta == " << delta);
    condDebugMsg(is_doorway, "map:get_next_move", "The traverse from current " << current << " to connector " << doorway_connector << 
		 " at " << connector << " goes thru doorway at region " << region_id);
    condDebugMsg(!is_doorway, "map:get_next_move", "The traverse from current " << current << " to connector " << doorway_connector << 
		 " at " << connector << " does not go thru doorway at region " << region_id);
    */
    return is_doorway;
  }

  unsigned int TopologicalMapAdapter::getOutletByPosition(double x, double y){
    unsigned int nearest_outlet_id = TopologicalMapAdapter::instance()->getNearestOutlet(x, y);
    robot_msgs::Pose outlet_pose;
    TopologicalMapAdapter::instance()->getOutletState(nearest_outlet_id, outlet_pose);
    double distance_to_outlet = sqrt(pow(outlet_pose.position.x - x, 2) + pow(outlet_pose.position.y - y, 2));
    debugMsg("map:get_next_move", "cost from connector to nearest outlet is " << distance_to_outlet);

    if(distance_to_outlet < 0.5){
      debugMsg("map:get_next_move", "Outlet found for <" << x << ", " << y << ">");
      return nearest_outlet_id;
    }

    return 0;
  }

  bool TopologicalMapAdapter::isDoorwayConnector(unsigned int connector_id){
    unsigned int region_a(0), region_b(0);
    bool doorway_a(false), doorway_b(false);
    try{
      getConnectorRegions(connector_id, region_a, region_b);
      isDoorway(region_a, doorway_a);
      isDoorway(region_b, doorway_b);
    }
    catch(...){
    }

    return doorway_a || doorway_b;
  }

  unsigned int TopologicalMapAdapter::getNextConnector(double x1, double y1, double x2, double y2){
    topological_map::Point2D p1(x1, y1);
    topological_map::Point2D p2(x2, y2);
    topological_map::ConnectorIdVector path = _map->shortestConnectorPath(p1, p2);
    ROS_ASSERT(path.size() >= 2);

    debugMsg("map:get_next_move", "Shortest path from <" << x1 << ", " << y1 << "> to <" << x2 << ", " << y2 << "> is " << pathToString(path));
    unsigned int next_connector = path[1];

    // The next connector is insuffient if we are already close to it. In the case that it is a door connecto we have to check against
    // the approach point of the connector. Otherwise we can check against the connector itself.
    robot_msgs::Pose pose;
    if(isDoorwayConnector(next_connector)){
      getDoorApproachPose(next_connector, pose);
    }
    else {
      getConnectorPosition(next_connector, pose.position.x, pose.position.y);
    }

    // We can take this next connector, except for the case where it's approach point is already really close to where we are. This check
    // only occurs where there are additional connectors to choose from and where the approach point for the connector is within our navigation tolerance. This
    // should also only arise if the connector is on a doorway
    bool too_close = fabs(pose.position.x - x1) < 0.1 && fabs(pose.position.y - y1) < 0.1;
    if(too_close){
      if(path.size() > 3){
	debugMsg("map:get_next_move", "Switching to successor as we are already at approach <" << pose.position.x << ", " << pose.position.y << "> for connector " << next_connector);
	next_connector = path[2];
      }
      else {
	debugMsg("map:get_next_move", "Next connector is already achieved so skip it <" << pose.position.x << ", " << pose.position.y << "> for connector " << next_connector);
	next_connector = 0;
      }
    }

    return next_connector;
  }

  std::string TopologicalMapAdapter::pathToString(const topological_map::ConnectorIdVector& path){
    std::stringstream ss;
    for(topological_map::ConnectorIdVector::const_iterator it = path.begin(); it != path.end(); ++it){
      ss << *it << "->";
    }

    ss << "!";
    return ss.str();
  }

  //*******************************************************************************************
  MapGetNextMoveConstraint::MapGetNextMoveConstraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _next_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[0]))),
     _next_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[1]))),
     _next_z(static_cast<AbstractDomain&>(getCurrentDomain(variables[2]))),
     _next_qx(static_cast<AbstractDomain&>(getCurrentDomain(variables[3]))),
     _next_qy(static_cast<AbstractDomain&>(getCurrentDomain(variables[4]))),
     _next_qz(static_cast<AbstractDomain&>(getCurrentDomain(variables[5]))),
     _next_qw(static_cast<AbstractDomain&>(getCurrentDomain(variables[6]))),
     _thru_doorway(static_cast<BoolDomain&>(getCurrentDomain(variables[7]))),
     _current_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[8]))),
     _current_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[9]))),
     _target_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[10]))),
     _target_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[11]))){
    checkError(variables.size() == 12, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  //*******************************************************************************************
  void MapGetNextMoveConstraint::handleExecute(){
    // Wait till inputs are bound. 
    if(!_current_x.isSingleton() || !_current_y.isSingleton() || !_target_x.isSingleton() || !_target_y.isSingleton()) {
      debugMsg("map:get_next_move",  "Exiting as inputs are not all bound");
      return;
    }

    // If ouputs are already bound, then do nothing
    static const std::string OUTPUT_PARAMS(":x:y:z:qx:qy:qz:qw");
    if(allSingletons(getScope(), OUTPUT_PARAMS)){
      debugMsg("map:get_next_move",  "Exiting as outputs are all bound");
      return;
    }

    debugMsg("map:get_next_move",  "BEFORE: "  << TREX::timeString() << toString());

    // Obtain the values
    double current_x = _current_x.getSingletonValue();
    double current_y = _current_y.getSingletonValue();
    double target_x = _target_x.getSingletonValue();
    double target_y = _target_y.getSingletonValue();
    unsigned int next_connector = TopologicalMapAdapter::instance()->getNextConnector(current_x, current_y, target_x, target_y);

    robot_msgs::Pose next_pose;
    bool is_doorway_connector = (next_connector > 0 ? TopologicalMapAdapter::instance()->isDoorwayConnector(next_connector) : false);
    if(is_doorway_connector){
      TopologicalMapAdapter::instance()->getDoorApproachPose(next_connector, next_pose);
      debugMsg("map:get_next_move",  "Selecting approach for doorway connector " << next_connector << " at <" << next_pose.position.x << ", " << next_pose.position.y << ">.");
    }
    else {
      unsigned int outlet_id = TopologicalMapAdapter::instance()->getOutletByPosition(target_x, target_y);

      if(outlet_id > 0){
	TopologicalMapAdapter::instance()->getOutletApproachPose(outlet_id, next_pose);
	debugMsg("map:get_next_move",  "Selecting approach for outlet connector " << next_connector << " at <" << next_pose.position.x << ", " << next_pose.position.y << ">.");
      }
      else {
	next_pose.position.x = target_x;
	next_pose.position.y = target_y;
	debugMsg("map:get_next_move",  "Going straight for target at <" << next_pose.position.x << ", " << next_pose.position.y << ">.");
      }
    }

    bool thru_doorway = TopologicalMapAdapter::instance()->isDoorway(current_x, current_y, next_connector);

    unsigned int next_region = TopologicalMapAdapter::instance()->getRegion(next_pose.position.x, next_pose.position.y);

    debugMsg("map:get_next_move", "next move is in region " << next_region);

    _next_x.set(next_pose.position.x);
    _next_y.set(next_pose.position.y);
    _next_z.set(next_pose.position.z);
    _next_qx.set(next_pose.orientation.x);
    _next_qy.set(next_pose.orientation.y);
    _next_qz.set(next_pose.orientation.z);
    _next_qw.set(next_pose.orientation.w);
    _thru_doorway.set(thru_doorway);

    debugMsg("map:get_next_move",  "AFTER: "  << TREX::timeString() << toString());
  }
}

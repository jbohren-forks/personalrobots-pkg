/**
 * @author Conor McGann
 */
#include <executive_trex_pr2/topological_map.h>
#include <ros/console.h>
#include "ConstrainedVariable.hh"
#include "Utilities.hh"
#include "Token.hh"
#include "Domains.hh"

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {


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

    debugMsg("map:get_nearest_outlet",  "BEFORE: " << toString());

    double x = _x.getSingletonValue();
    double y = _y.getSingletonValue();
    unsigned int outlet_id = TopologicalMapAdapter::instance()->getNearestOutlet(x, y);

    checkError(outlet_id > 0, "No outlet found for <" << x << ", " << y << ">");

    _outlet.set(outlet_id);

    debugMsg("map:get_nearest_outlet",  "AFTER: " << toString());
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

    debugMsg("map:get_outlet_state",  "BEFORE: " << toString());
    robot_msgs::Pose outlet_pose;
    TopologicalMapAdapter::instance()->getOutletState(_outlet.getSingletonValue(), outlet_pose);
    _x.set(outlet_pose.position.x);
    _y.set(outlet_pose.position.y);
    _z.set(outlet_pose.position.z);
    _qx.set(outlet_pose.orientation.x);
    _qy.set(outlet_pose.orientation.y);
    _qz.set(outlet_pose.orientation.z);
    _qw.set(outlet_pose.orientation.w);

    debugMsg("map:get_outlet_state",  "AFTER: " << toString());
  }

  //*******************************************************************************************
  MapGetNextMoveConstraint::MapGetNextMoveConstraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _next_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[0]))),
     _next_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[1]))),
     _thru_doorway(static_cast<BoolDomain&>(getCurrentDomain(variables[2]))),
     _current_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[3]))),
     _current_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[4]))),
     _target_x(static_cast<AbstractDomain&>(getCurrentDomain(variables[5]))),
     _target_y(static_cast<AbstractDomain&>(getCurrentDomain(variables[6]))){
    checkError(variables.size() == 7, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  //*******************************************************************************************
  void MapGetNextMoveConstraint::handleExecute(){
    // Wait till inputs are bound. 
    if(!_current_x.isSingleton() || !_current_x.isSingleton() || !_target_x.isSingleton() || !_target_y.isSingleton()) 
      return;

    debugMsg("map:get_next_move",  "BEFORE: " << toString());


    // Get next move by evaluating all options
    double lowest_cost = PLUS_INFINITY;
    double next_x, next_y;

    // Obtain the values
    double current_x = _current_x.getSingletonValue();
    double current_y = _current_y.getSingletonValue();
    double target_x = _target_x.getSingletonValue();
    double target_y = _target_y.getSingletonValue();

    unsigned int this_region =  TopologicalMapAdapter::instance()->getRegion(current_x, current_y);
    condDebugMsg(this_region == 0, "map", "No region for <" << current_x << ", " << current_y <<">");
    unsigned int final_region =  TopologicalMapAdapter::instance()->getRegion(target_x, target_y);
    condDebugMsg(final_region == 0, "map", "No region for <" << target_x << ", " << target_y <<">");

    // If the final region is bogus, then 
    if(final_region == 0 || this_region == 0){
      _thru_doorway.empty();
      return;
    }

    // If the source point and final point can be connected without then we consider an option of going
    // directly to the point rather than thru a connector.
    if(this_region == final_region){
      lowest_cost = TopologicalMapAdapter::instance()->cost(current_x, current_y, target_x, target_y);
      next_x = target_x;
      next_y = target_y;
    }

    std::vector< std::pair<topological_map::ConnectorId, double> > connector_cost_pairs;
    ros::Time before = ros::Time::now();
    TopologicalMapAdapter::instance()->getConnectorCosts(current_x, current_y, target_x, target_y, connector_cost_pairs);
    ros::Duration elapsed = ros::Time::now() - before;
    debugMsg("map:get_next_move", "connection_cost_latency is " << elapsed.toSec());

    for(std::vector< std::pair<topological_map::ConnectorId, double> >::const_iterator it = connector_cost_pairs.begin(); it != connector_cost_pairs.end(); ++it){
      if(it->second < lowest_cost){
	lowest_cost = it->second;
	TopologicalMapAdapter::instance()->getConnectorPosition(it->first, next_x, next_y);
      }
    }

    if(lowest_cost == PLUS_INFINITY){
      _thru_doorway.empty();
    }
    else {
      _next_x.set(next_x);
      _next_y.set(next_y);
    }

    // Finally, we have to set the flag for if we are moving thru a doorway
    double distance = sqrt(pow(current_x - next_x, 2) + pow(current_y - next_y, 2));
    bool is_doorway(false);
    // If points are pretty close, check if it is a doorway
    if(distance < 3){
      double mid_x = (current_x + next_x) / 2;
      double mid_y = (current_y + next_y) / 2;
      unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(mid_x, mid_y);
      if(!TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway)){
	_thru_doorway.empty();
      }
    }

    _thru_doorway.set(is_doorway);

    debugMsg("map:get_next_move",  "AFTER: " << toString());
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
    debugMsg("map:connector",  "BEFORE: " << toString());
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
    debugMsg("map:get_connector",  "AFTER: " << toString());
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

    debugMsg("map:get_region_from_position",  "BEFORE: " << toString());

    double x = _x.getSingletonValue();
    double y = _y.getSingletonValue();

    unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(x, y);
    _region.set(region_id);

    debugMsg("map:get_region_from_position",  "AFTER: " << toString());

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

    debugMsg("map:get_doorway_from_points",  "BEFORE: " << toString());

    double x1 = _x1.getSingletonValue();
    double y1 = _y1.getSingletonValue();
    double x2 = _x2.getSingletonValue();
    double y2 = _y2.getSingletonValue();
    double x = (x1 + x2) / 2;
    double y = (y1 + y2) / 2;
    unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(x, y);

    checkError(region_id > 0, "No region at midpoint given by <" << x << ", " << y << ">");

    condDebugMsg(region_id == 0, "map:get_doorway_from_points", 
		 "isObstacle(" << x << ", " << y << ") == " << (TopologicalMapAdapter::instance()->isObstacle(x, y) ? "TRUE" : "FALSE"));

    // Check if it is a doorway
    bool is_doorway(true);
    TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway);


    condDebugMsg(is_doorway == false, "map:get_doorway_from_points", "region " << region_id << "containing <" << x << ", " << y << "> is not a doorway");

    // If it is not a doorway, then set to zero
    if(is_doorway == false)
      _region.set(0);
    else
      _region.set(region_id);

    debugMsg("map:get_doorway_from_points",  "AFTER: " << toString());
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

    debugMsg("map:is_doorway",  "BEFORE: " << toString());

    unsigned int region_id = _region.getSingletonValue();
    bool is_doorway(true);

    TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway);
    _result.set(is_doorway);

    debugMsg("map:is_doorway",  "AFTER: " << toString());
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

    debugMsg("map:get_door_state",  "BEFORE: " << toString());

    unsigned int id = (unsigned int)_door_id.getSingletonValue();

    robot_msgs::Door door_state;
    if(!TopologicalMapAdapter::instance()->getDoorState(id, door_state))
      _door_id.empty();
    else {
      static const LabelStr MAP_FRAME("map");
      double timestamp = ros::Time::now().toSec();
      if(!apply(MAP_FRAME, "frame_id") ||
	 !apply(timestamp, "time_stamp") ||
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
	 !apply(door_state.handle.z, "handle_z")) {
      }
    }
      
    debugMsg("map:get_door_state",  "AFTER: " << toString());
  }
  
  std::string MapGetDoorStateConstraint::toString() const {
    std::stringstream ss;
    for(std::vector<ConstrainedVariableId>::const_iterator it = _token_id->parameters().begin(); it != _token_id->parameters().end(); ++it){
      ConstrainedVariableId var = *it;
      ss << var->getName().toString() << " == " << var->lastDomain().toString() << std::endl;
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

    _map = topological_map::TopologicalMapPtr(new topological_map::TopologicalMap(in));

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

  bool TopologicalMapAdapter::getDoorState(unsigned int doorway_id, robot_msgs::Door& door_state){
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
    results = _map->connectorCosts(source_point, target_point);
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

  /**
   * @todo Outlet data should be a pose message
   */
  void TopologicalMapAdapter::getOutletState(unsigned int outlet_id, robot_msgs::Pose& outlet_pose){
    topological_map::OutletInfo outlet_info = _map->outletInfo(outlet_id);
    outlet_pose.position.x = outlet_info.origin_x;
    outlet_pose.position.y = outlet_info.origin_y;
    outlet_pose.position.z = outlet_info.origin_z;
    outlet_pose.orientation.x = 0.0;
    outlet_pose.orientation.y = 0.0;
    outlet_pose.orientation.z = 0.0;
    outlet_pose.orientation.w = 1.0;
  }

  void TopologicalMapAdapter::observeOutletBlocked(unsigned int outlet_id){
    _map->observeOutletBlocked(outlet_id);
  }
}

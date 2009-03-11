/**
 * @author Conor McGann
 */
#include <executive_trex_pr2/topological_map.h>
#include "ConstrainedVariable.hh"
#include "Token.hh"

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {

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
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(_x.getSingletonValue(), _y.getSingletonValue());
      _region.set(region_id);
    }
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
    if(_region.isSingleton()){
      unsigned int region_id = _region.getSingletonValue();
      bool is_doorway(true);

      // If the region is bogus then this is an inconsistency. Otherwise bind the result
      if(!TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway))
	_region.empty();
      else
	_result.set(is_doorway);
    }
  }

    
  //*******************************************************************************************
  MapGetDoorFromPositionConstraint::MapGetDoorFromPositionConstraint(const LabelStr& name,
								     const LabelStr& propagatorName,
								     const ConstraintEngineId& constraintEngine,
								     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _door(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))){
    checkError(variables.size() == 5, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  void MapGetDoorFromPositionConstraint::handleExecute(){
    if(_x1.isSingleton() && _y1.isSingleton() && _x2.isSingleton() && _y2.isSingleton()){
      unsigned int door_id = TopologicalMapAdapter::instance()->getDoorFromPosition(_x1.getSingletonValue(),
										     _y1.getSingletonValue(),
										     _x2.getSingletonValue(),
										     _y2.getSingletonValue());
      _door.set(door_id);
    }
  }


    
  //*******************************************************************************************
  MapGetDoorDataConstraint::MapGetDoorDataConstraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _door(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[4]))){
    checkError(variables.size() == 5, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }

  void MapGetDoorDataConstraint::handleExecute(){
    if(_door.isSingleton() && _door.getSingletonValue() > 0){
      double x1, y1, x2, y2;
      bool result = TopologicalMapAdapter::instance()->getDoorData(x1, y1, x2, y2, _door.getSingletonValue());
      
      // If this was not a valid id for some reason, I am stunned! But we should generate an inconsistency
      if(!result)
	_door.empty();
      else {
	_x1.set(x1);
	_y1.set(y1);
	_x2.set(x2);
	_y2.set(y2);
      }
    }
  }


    
  //*******************************************************************************************
  MapGetHandlePositionConstraint::MapGetHandlePositionConstraint(const LabelStr& name,
								 const LabelStr& propagatorName,
								 const ConstraintEngineId& constraintEngine,
								 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _z(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _door(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[3]))){
    checkError(variables.size() == 4, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }

  /**
   * @todo This implementation is bogus. It should be querying other api
   */
  void MapGetHandlePositionConstraint::handleExecute(){
    if(_door.isSingleton() && _door.getSingletonValue() > 0){
      double x1, y1, x2, y2;
      bool result = TopologicalMapAdapter::instance()->getDoorData(x1, y1, x2, y2, _door.getSingletonValue());
      
      // If this was not a valid id for some reason, I am stunned! But we should generate an inconsistency
      if(!result)
	_door.empty();
      else {
	_x.set(x1);
	_y.set(y1);
	_z.set(1.0);
      }
    }
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

    unsigned int this_region =  TopologicalMapAdapter::instance()->getRegion(x0, y0);
    condDebugMsg(this_region == 0, "MapConnectorSelector", "No region for <" << x0 << ", " << y0 <<">");
    unsigned int final_region =  TopologicalMapAdapter::instance()->getRegion(x1, y1);
    condDebugMsg(final_region == 0, "MapConnectorSelector", "No region for <" << x1 << ", " << y1 <<">");
    std::vector<unsigned int> final_region_connectors;
    TopologicalMapAdapter::instance()->getRegionConnectors(final_region, final_region_connectors);

    // exclude the source connector if there is one
    bool accessible_directly(false);

    unsigned int source_connector = TopologicalMapAdapter::instance()->getConnector(x0, y0);
    for(unsigned int i = 0; i < final_region_connectors.size(); i++){
      if(final_region_connectors[i] == source_connector){
	accessible_directly = true;
	break;
      }
    }

    // If the source point and final point can be connected then we consider an option of going
    // directly to the point rather than thru a connector. To figure this out we should look the region at
    // the source connector and see if it is a connector for the final region
    if(accessible_directly)
      _sorted_choices.push_back(Choice(0, sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2))));

    // Now iterate over the connectors in this region and compute the heuristic cost estimate
    std::vector<unsigned int> connectors;
    TopologicalMapAdapter::instance()->getRegionConnectors(this_region, connectors);
    for(std::vector<unsigned int>::const_iterator it = connectors.begin(); it != connectors.end(); ++it){
      unsigned int connector_id = *it;
      if(connector_id != source_connector){
	double cost = TopologicalMapAdapter::instance()->gCost(x0, y0, connector_id) + 
	  TopologicalMapAdapter::instance()->hCost(connector_id, x1, y1);

	_sorted_choices.push_back(Choice(connector_id, cost));
      }
    }

    _sorted_choices.sort();
    _choice_iterator = _sorted_choices.begin();

    debugMsg("MapConnectorSelector:MapConnectorSelector", 
	     "Evaluating from <" << x0 << ", " << y0 << "> to <" << x1 << ", " << y1 << ">. " << toString());
  }

  std::string MapConnectorSelector::toString() const {
    std::stringstream ss;
    ss << "Choices are: " << std::endl;
    for(std::list<Choice>::const_iterator it = _sorted_choices.begin(); it != _sorted_choices.end(); ++it){
      const Choice& choice = *it;
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
    MapConnectorSelector::Choice c = *_choice_iterator;
    debugMsg("MapConnectorSelector::getNext", "Selecting " << c.id << std::endl);
    ++_choice_iterator;
    return c.id;
  }

  double MapConnectorSelector::gCost(unsigned int from, unsigned int to) const{
    return 0;
  }

  double MapConnectorSelector::hCost(unsigned int from, unsigned int to) const{
    return 0;
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

    toPostScriptFile();
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

  /**
   * @todo Fix to integrate with actual world model data
   */
  unsigned int TopologicalMapAdapter::getDoorFromPosition(double x1, double y1, double x2, double y2){
    return 1;
  }

  /**
   * @todo Fix to integrate with actual world model data
   */
  bool TopologicalMapAdapter::getDoorData(double& x1, double& y1, double& x2, double& y2, unsigned int door_id){
    x1 = 2;
    y1 = 5;
    x2 = 4;
    y2 = 5;
    return true;
  }


  bool TopologicalMapAdapter::isDoorway(unsigned int region_id, bool& result){
    try{
      int region_type = _map->regionType(region_id);
      result = (region_type ==  topological_map::DOORWAY);
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::isObstacle(double x, double y) {
    return _map->isObstacle(topological_map::Point2D(x, y));
  }

  double TopologicalMapAdapter::gCost(double from_x, double from_y, unsigned int connector_id){
    double x(0), y(0);

    // If there is no connector for this id then we have an infinite cost
    if(!getConnectorPosition(connector_id, x, y))
      return PLUS_INFINITY;

    // Otherwise we return the euclidean distance between source and target
    return sqrt(pow(from_x - x, 2) + pow(from_y - y, 2));
  }

  double TopologicalMapAdapter::hCost(unsigned int connector_id, double to_x, double to_y){
    // For now, use g_cost approach
    return gCost(to_x, to_y, connector_id);
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
}

/**
 * @author Conor McGann
 */
#include <executive_trex_pr2/topological_map.h>
#include "ConstrainedVariable.hh"
#include "Token.hh"

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {

  /**
   * A Default Grid Structure to sue when testing
   */
  void setV (topological_map::OccupancyGrid& grid, unsigned r0, unsigned dr, unsigned rmax, unsigned c0, unsigned dc, unsigned cmax, bool val) 
  {
    for (unsigned r=r0; r<rmax; r+=dr) {
      for (unsigned c=c0; c<cmax; c+=dc) {
	grid[r][c] = val;
      }
    }
  }

  // Get a sample grid:
  // Dimensions: 21 * 24
  // Resolution: 1
  static const unsigned int HEIGHT_21(21);
  static const unsigned int WIDTH_24(24);
  topological_map::OccupancyGrid& GRID_3_3_ALL_CONNECTED(){
    static topological_map::OccupancyGrid grid(boost::extents[HEIGHT_21][WIDTH_24]);
    static bool initialized(false);
    if(!initialized){
      setV(grid, 0, 1, HEIGHT_21, 0, 1, WIDTH_24, false);
      setV(grid, 7, 7, HEIGHT_21, 0, 1, WIDTH_24, true);
      setV(grid, 0, 1, HEIGHT_21, 8, 8, WIDTH_24, true);
      setV(grid, 3, 7, HEIGHT_21, 8, 8, WIDTH_24, false);
      setV(grid, 7, 7, HEIGHT_21, 4, 8, WIDTH_24, false);
      initialized = true;
    }

    return grid;
  }

// Shared setup for all tests
  TopologicalMapAdapter* map = new TopologicalMapAdapter(GRID_3_3_ALL_CONNECTED(), 1.0);

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
    checkError(TopologicalMapAccessor::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
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
      if(TopologicalMapAccessor::instance()->getConnectorPosition(connector_id, x, y)){
	_x.set(x);

	if(!_x.isEmpty()) 
	  _y.set(y);
      }
    }

    // If x and y are set, we can bind the connector.
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int connector_id = TopologicalMapAccessor::instance()->getConnector(_x.getSingletonValue(), _y.getSingletonValue());
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
    checkError(TopologicalMapAccessor::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   */
  void MapGetRegionFromPositionConstraint::handleExecute(){
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int region_id = TopologicalMapAccessor::instance()->getRegion(_x.getSingletonValue(), _y.getSingletonValue());
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
    checkError(TopologicalMapAccessor::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  void MapIsDoorwayConstraint::handleExecute(){
    if(_region.isSingleton()){
      unsigned int region_id = _region.getSingletonValue();
      bool is_doorway(true);

      // If the region is bogus then this is an inconsistency. Otherwise bind the result
      if(!TopologicalMapAccessor::instance()->isDoorway(region_id, is_doorway))
	_region.empty();
      else
	_result.set(is_doorway);
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

    unsigned int this_region = TopologicalMapAccessor::instance()->getRegion(x0, y0);  
    checkError(this_region > 0, "No region for <" << x0 << ", " << y0 <<">");
    unsigned int final_region =  TopologicalMapAccessor::instance()->getRegion(x1, y1);
    checkError(final_region > 0, "No region for <" << x1 << ", " << y1 <<">");

    // Build the set of choices based on all connectors for the current region
    std::vector<unsigned int> connectors;
    TopologicalMapAccessor::instance()->getRegionConnectors(this_region, connectors);

    // If the current region and the target region are the same, include the 0 region id, which
    // indicates we travel within the region
    if(this_region == final_region)
      _sorted_choices.push_back(Choice(0, sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2))));

    // exclude the source connector if there is one
    unsigned int source_connector = TopologicalMapAccessor::instance()->getConnector(x0, y0);

    // Now iterate over the connectors in this region and compute the heuristic cost estimate
    for(std::vector<unsigned int>::const_iterator it = connectors.begin(); it != connectors.end(); ++it){
      unsigned int connector_id = *it;
      if(connector_id != source_connector){
	double cost = TopologicalMapAccessor::instance()->gCost(x0, y0, connector_id) + 
	  TopologicalMapAccessor::instance()->hCost(connector_id, x1, y1);

	_sorted_choices.push_back(Choice(connector_id, cost));
      }
    }

    _sorted_choices.sort();
    _choice_iterator = _sorted_choices.begin();
  }

  bool MapConnectorSelector::hasNext() const { return _choice_iterator != _sorted_choices.end(); }

  double MapConnectorSelector::getNext(){
    MapConnectorSelector::Choice c = *_choice_iterator;
    ++_choice_iterator;
    return c.id;
  }

  double MapConnectorSelector::gCost(unsigned int from, unsigned int to) const{
    return 0;
  }

  double MapConnectorSelector::hCost(unsigned int from, unsigned int to) const{
    return 0;
  }

  TopologicalMapAccessor* TopologicalMapAccessor::_singleton = NULL;

  /**
   * Topological Map Accessor Implementation. Note that it is basically just an interface.
   */
  TopologicalMapAccessor::TopologicalMapAccessor(double resolution, unsigned int c, unsigned int r)
    :_resolution(resolution), _num_cols(c), _num_rows(r){
    if(_singleton != NULL)
      delete _singleton;

    _singleton = this;
  }

  TopologicalMapAccessor::~TopologicalMapAccessor(){
    _singleton = NULL;
  }

  TopologicalMapAccessor* TopologicalMapAccessor::instance(){
    return _singleton;
  }

  /************************************************************************
   * Map Adapter implementation
   ************************************************************************/

  TopologicalMapAdapter::TopologicalMapAdapter(const topological_map::OccupancyGrid& grid, double resolution)
    : TopologicalMapAccessor(resolution, topological_map::numCols(grid), topological_map::numRows(grid)){
    _map = topological_map::topologicalMapFromGrid(grid, resolution, 2, 1, 1, 0, "local");
  }

  TopologicalMapAdapter::~TopologicalMapAdapter(){

  }

  unsigned int TopologicalMapAdapter::getRegion(double x, double y){
    unsigned int result = 0;
    try{
      result = _map->containingRegion(topological_map::Point2D(x, y));
    }
    catch(...){}
    return result;
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
}

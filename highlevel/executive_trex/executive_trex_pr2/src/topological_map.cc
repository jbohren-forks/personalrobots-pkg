/**
 * @author Conor McGann
 */

#include <executive_trex_pr2/topological_map.h>
#include "ConstrainedVariable.hh"
#include "Token.hh"
//#include <map_server/image_loader.h>

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {

  /*

// A function: given an x,y position, bind a region.
constraint map_get_region_from_position(region, x, y){ region <: Region && x <: numeric && y <: numeric }

// A relation: constrain two connection variables so that they share a common region. Propagates when one of 3 is bound.
constraint map_connected(connection_a, connection_b, region){ connection_a <: Connector && connection_b <: Connector && region <: Region }

// A function to query if a region is a doorway
constraint map_is_doorway(result, region) { result <: bool && region <: Region }
  */
  //*******************************************************************************************
  map_connector_constraint::map_connector_constraint(const LabelStr& name,
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
  void map_connector_constraint::handleExecute(){
    if(_connector.isSingleton()){
      unsigned int connector_id = (unsigned int) _connector.getSingletonValue();
      double x, y;

      // If we have a pose, bind the variables
      if(TopologicalMapAccessor::instance()->get_connector_position(connector_id, x, y)){
	_x.set(x);

	if(!_x.isEmpty()) 
	  _y.set(y);
      }
    }

    // If x and y are set, we can bind the connector.
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int connector_id = TopologicalMapAccessor::instance()->get_connector(_x.getSingletonValue(), _y.getSingletonValue());
      _connector.set(connector_id);
    }
  }

  //*******************************************************************************************
  map_get_region_from_position_constraint::map_get_region_from_position_constraint(const LabelStr& name,
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
  void map_get_region_from_position_constraint::handleExecute(){
    if(_x.isSingleton() && _y.isSingleton()){
      unsigned int region_id = TopologicalMapAccessor::instance()->get_region(_x.getSingletonValue(), _y.getSingletonValue());
      _region.set(region_id);
    }
  }
    

  //*******************************************************************************************
  map_connected_constraint::map_connected_constraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  void map_connected_constraint::handleExecute(){}
    
    
  //*******************************************************************************************
  map_is_doorway_constraint::map_is_doorway_constraint(const LabelStr& name,
						       const LabelStr& propagatorName,
						       const ConstraintEngineId& constraintEngine,
						       const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  void map_is_doorway_constraint::handleExecute(){}
    


  //*******************************************************************************************
  map_connector_filter::map_connector_filter(const TiXmlElement& configData)
    : SOLVERS::FlawFilter(configData, true),
      _source(extractData(configData, "source")),
      _final(extractData(configData, "final")),
      _target(extractData(configData, "target")){}

  /**
   * The filter will exclude the entity if it is not the target variable of a token with the right structure
   * with all inputs bound so that the selection can be made based on sufficient data
   */
  bool map_connector_filter::test(const EntityId& entity){
    // It should be a variable
    if(!ConstrainedVariableId::convertable(entity))
      return true;

    // Its name should match the target, it should be enumerated, and it should not be a singleton.
    ConstrainedVariableId var = (ConstrainedVariableId) entity;
    if(var->getName() != _target || var->lastDomain().isEnumerated() || var->lastDomain().isSingleton())
      return true;

    // It should have a parent that is a token
    if(var->parent().isNoId() || !TokenId::convertable(var->parent()))
      return true;

    TokenId parent = (TokenId) var->parent();
    ConstrainedVariableId source = parent->getVariable(_source, false);
    ConstrainedVariableId final = parent->getVariable(_final, false);

    return source.isNoId() || !source->lastDomain().isSingleton() || final.isNoId() || !final->lastDomain().isSingleton();
  }


  //*******************************************************************************************
  map_connector_selector::map_connector_selector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation)
    : SOLVERS::UnboundVariableDecisionPoint(client, flawedVariable, configData, explanation),
      _source(extractData(configData, "source")),
      _final(extractData(configData, "final")){

    // Verify expected structure for the variable - wrt other token parameters
    checkError(flawedVariable->parent().isId() && TokenId::convertable(flawedVariable->parent()), 
	       "The variable configuration is incorrect. The map_connector_filter must not be working. Expect a token parameter with source connector and final region.");


    // Obtain source and final 
    TokenId parent = (TokenId) flawedVariable->parent();
    ConstrainedVariableId source = parent->getVariable(_source, false);
    ConstrainedVariableId final = parent->getVariable(_final, false);

    // Double check all the inputs
    checkError(source.isId(), "Bad filter. Invalid source name: " << _source.toString() << " on token " << parent->toString());
    checkError(final.isId(), "Bad filter. Invalid final name: " << _final.toString() << " on token " << parent->toString());
    checkError(source->lastDomain().isSingleton(), "Bad Filter. Source is not a singleton: " << source->toString());
    checkError(final->lastDomain().isSingleton(), "Bad Filter. Final is not a singleton: " << final->toString());

    // Compose all the choices.
    const EnumeratedDomain& dom = static_cast<const EnumeratedDomain&>(flawedVariable->lastDomain());
    for(std::set<double>::const_iterator it = dom.getValues().begin(); it != dom.getValues().end(); ++it){
      Choice choice;
      double value = *it;
      unsigned int id = (unsigned int) value;
      checkError(value == id, value << " != " << id);
      choice.cost = g_cost((unsigned int) source->lastDomain().getSingletonValue(), id) + h_cost(id, (unsigned int) final->lastDomain().getSingletonValue());
      choice.id = id;
      _sorted_choices.push_back(choice);
    }

    _sorted_choices.sort();
    _choice_iterator = _sorted_choices.begin();
  }

  bool map_connector_selector::hasNext() const { return _choice_iterator == _sorted_choices.end(); }

  double map_connector_selector::getNext(){
    map_connector_selector::Choice c = *_choice_iterator;
    ++_choice_iterator;
    return c.id;
  }

  double map_connector_selector::g_cost(unsigned int from, unsigned int to) const{
    return 0;
  }

  double map_connector_selector::h_cost(unsigned int from, unsigned int to) const{
    return 0;
  }


  TopologicalMapAccessor* TopologicalMapAccessor::_singleton = NULL;

  /**
   * Topological Map Accessor Implementation. Note that it is basically just an interface.
   */
  TopologicalMapAccessor::TopologicalMapAccessor(){
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
  TopologicalMapAdapter::TopologicalMapAdapter(const std::string& map_file_name){
    //robot_srvs::StaticMap::Response resp;
    //loadMapFromFile(&resp, map_file_name.c_str(), 0.25, false);
  }

  TopologicalMapAdapter::~TopologicalMapAdapter(){
  }


  unsigned int TopologicalMapAdapter::get_region(double x, double y){
    unsigned int result = 0;
    try{
      result = _map->containingRegion(topological_map::Point2D(x, y));
    }
    catch(...){}
    return result;
  }

  unsigned int TopologicalMapAdapter::get_connector(double x, double y){
    unsigned int result = 0;
    try{
      result = _map->pointConnector(topological_map::Point2D(x, y));
    }
    catch(...){}
    return result;
  }

  bool TopologicalMapAdapter::get_connector_position(unsigned int connector_id, double& x, double& y){
    try{
      topological_map::Point2D p = _map->connectorPosition(connector_id);
      x = p.x;
      y = p.y;
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::get_region_connectors(unsigned int region_id, std::vector<unsigned int>& connectors){
    try{
      connectors = _map->adjacentConnectors(region_id);
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::get_connector_regions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b){
    try{
      topological_map::RegionPair pair = _map->adjacentRegions(connector_id);
      region_a = pair.first;
      region_b = pair.second;
      return true;
    }
    catch(...){}
    return false;
  }

  bool TopologicalMapAdapter::is_doorway(unsigned int region_id, bool& result){
    try{
      int region_type = _map->regionType(region_id);
      result = (region_type ==  topological_map::DOORWAY);
      return true;
    }
    catch(...){}
    return false;
  }

  double TopologicalMapAdapter::get_g_cost(double from_x, double from_y, unsigned int connector_id){
    double x(0), y(0);

    // If there is no connector for this id then we have an infinite cost
    if(!get_connector_position(connector_id, x, y))
      return PLUS_INFINITY;

    // Otherwise we return the euclidean distance between source and target
    return sqrt(pow(from_x - x, 2) + pow(from_y - y, 2));
  }

  double TopologicalMapAdapter::get_h_cost(unsigned int connector_id, double to_x, double to_y){
    // For now, use g_cost approach
    return get_g_cost(to_x, to_y, connector_id);
  }
}

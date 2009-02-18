/**
 * @author Conor McGann
 */

#include <executive_trex_pr2/topological_map.h>
#include "ConstrainedVariable.hh"
#include "Token.hh"

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
     m_connector(static_cast<IntervalIntDomain&>(getCurrentDomain(variables[0]))),
     m_x(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     m_y(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     m_th(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))){
    checkError(variables.size() == 4, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAccessor::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  map_connector_constraint::~map_connector_constraint(){}
    
  /**
   * @brief When the connector id is bound, we bind the values for x, y, and th based on a lookup in the topological map.
   * We can also bind the value for the connector id based on x and y inputs. This enforces the full relation.
   */
  void map_connector_constraint::handleExecute(){
    if(m_connector.isSingleton()){
      unsigned int connector_id = (unsigned int) m_connector.getSingletonValue();
      double x, y, th;

      // If we have a pose, bind the variables
      if(TopologicalMapAccessor::instance()->get_connector_position(connector_id, x, y, th)){
	m_x.set(x);

	if(!m_x.isEmpty()) 
	  m_y.set(y);

	if(!m_y.isEmpty()) 
	  m_th.set(th);
      }
    }

    // If x and y are set, we can bind the connector.
    if(m_x.isSingleton() && m_y.isSingleton()){
      unsigned int connector_id = TopologicalMapAccessor::instance()->get_connector(m_x.getSingletonValue(), m_y.getSingletonValue());
      m_connector.set(connector_id);
    }
  }

  //*******************************************************************************************
  map_get_region_from_position_constraint::map_get_region_from_position_constraint(const LabelStr& name,
										   const LabelStr& propagatorName,
										   const ConstraintEngineId& constraintEngine,
										   const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  map_get_region_from_position_constraint::~map_get_region_from_position_constraint(){}
    
  void map_get_region_from_position_constraint::handleExecute(){}
    

  //*******************************************************************************************
  map_connected_constraint::map_connected_constraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  map_connected_constraint::~map_connected_constraint(){}
    
  void map_connected_constraint::handleExecute(){}
    
    
  //*******************************************************************************************
  map_is_doorway_constraint::map_is_doorway_constraint(const LabelStr& name,
						       const LabelStr& propagatorName,
						       const ConstraintEngineId& constraintEngine,
						       const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  map_is_doorway_constraint::~map_is_doorway_constraint(){}
    
  void map_is_doorway_constraint::handleExecute(){}
    


  //*******************************************************************************************
  map_connector_filter::map_connector_filter(const TiXmlElement& configData)
    : SOLVERS::FlawFilter(configData, true),
      m_source(extractData(configData, "source")),
      m_final(extractData(configData, "final")),
      m_target(extractData(configData, "target")){}

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
    if(var->getName() != m_target || var->lastDomain().isEnumerated() || var->lastDomain().isSingleton())
      return true;

    // It should have a parent that is a token
    if(var->parent().isNoId() || !TokenId::convertable(var->parent()))
      return true;

    TokenId parent = (TokenId) var->parent();
    ConstrainedVariableId source = parent->getVariable(m_source, false);
    ConstrainedVariableId final = parent->getVariable(m_final, false);

    return source.isNoId() || !source->lastDomain().isSingleton() || final.isNoId() || !final->lastDomain().isSingleton();
  }


  //*******************************************************************************************
  map_connector_selector::map_connector_selector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation)
    : SOLVERS::UnboundVariableDecisionPoint(client, flawedVariable, configData, explanation),
      m_source(extractData(configData, "source")),
      m_final(extractData(configData, "final")){

    // Verify expected structure for the variable - wrt other token parameters
    checkError(flawedVariable->parent().isId() && TokenId::convertable(flawedVariable->parent()), 
	       "The variable configuration is incorrect. The map_connector_filter must not be working. Expect a token parameter with source connector and final region.");


    // Obtain source and final 
    TokenId parent = (TokenId) flawedVariable->parent();
    ConstrainedVariableId source = parent->getVariable(m_source, false);
    ConstrainedVariableId final = parent->getVariable(m_final, false);

    // Double check all the inputs
    checkError(source.isId(), "Bad filter. Invalid source name: " << m_source.toString() << " on token " << parent->toString());
    checkError(final.isId(), "Bad filter. Invalid final name: " << m_final.toString() << " on token " << parent->toString());
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
      m_sorted_choices.push_back(choice);
    }

    m_sorted_choices.sort();
    m_choice_iterator = m_sorted_choices.begin();
  }

  bool map_connector_selector::hasNext() const { return m_choice_iterator == m_sorted_choices.end(); }

  double map_connector_selector::getNext(){
    map_connector_selector::Choice c = *m_choice_iterator;
    ++m_choice_iterator;
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
}

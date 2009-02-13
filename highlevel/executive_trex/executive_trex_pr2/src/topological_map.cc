/**
 * @author Conor McGann
 */

#include "topological_map.hh"
#include "ConstrainedVariable.hh"
#include "Token.hh"

using namespace EUROPA;

namespace TREX {
  
  //*******************************************************************************************
  map_connector_constraint::map_connector_constraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						     const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables){}
    
  map_connector_constraint::~map_connector_constraint(){}
    
  void map_connector_constraint::handleExecute(){}
    

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
}

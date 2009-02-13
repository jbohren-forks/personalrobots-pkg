/**
 * @author Conor McGann
 * @brief Implements bindings for constraints and other queries for accessing the topological map
 */

#ifndef _TOPOLOGICAL_MAP_H_
#define _TOPOLOGICAL_MAP_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "IntervalDomain.hh"
#include "IntervalIntDomain.hh"
#include "BoolDomain.hh"
#include "Logger.hh"
#include "FlawFilter.hh"
#include "UnboundVariableDecisionPoint.hh"

using namespace EUROPA;

namespace TREX {
  
  /**
   * @brief A relation: given a connector, bind the x, y the values. Given x, and y, bind the connector
   */
  class map_connector_constraint : public Constraint {
  public:
    
    map_connector_constraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~map_connector_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A function: given an x,y position, bind a region.
   */
  class map_get_region_from_position_constraint : public Constraint {
  public:
    
    map_get_region_from_position_constraint(const LabelStr& name,
					    const LabelStr& propagatorName,
					    const ConstraintEngineId& constraintEngine,
					    const std::vector<ConstrainedVariableId>& variables);
    
    ~map_get_region_from_position_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A relation: constrain two connection variables so that they share a common region. Propagates when one of 3 is bound.
   */
  class map_connected_constraint : public Constraint {
  public:
    
    map_connected_constraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~map_connected_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A function to query if a region is a doorway
   */
  class map_is_doorway_constraint : public Constraint {
  public:
    
    map_is_doorway_constraint(const LabelStr& name,
			      const LabelStr& propagatorName,
			      const ConstraintEngineId& constraintEngine,
			      const std::vector<ConstrainedVariableId>& variables);
    
    ~map_is_doorway_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A filter to exclude variable binding decisions unless they are on a parameter variable
   * of a token for has all required variables, and the source and final destination are already singletons
   */
  class map_connector_filter: public SOLVERS::FlawFilter {
  public:
    map_connector_filter(const TiXmlElement& configData);

    bool test(const EntityId& entity);

  private:
    const LabelStr m_source;
    const LabelStr m_final;
    const LabelStr m_target;
  };


  /**
   * @brief Implements a sort based on minimizing g_cost + h_cost for target selection
   */
  class map_connector_selector: public SOLVERS::UnboundVariableDecisionPoint {
  public:
    map_connector_selector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation = "unknown");
    bool hasNext() const;
    double getNext();

  private:
    class Choice {
    public:
      Choice():id(0), cost(0){}
      unsigned int id;
      double cost;  
      bool operator<(const map_connector_selector::Choice& rhs) const {return cost < rhs.cost;}
    };

    double g_cost(unsigned int from, unsigned int to) const;
    double h_cost(unsigned int from, unsigned int to) const;

    const LabelStr m_source;
    const LabelStr m_final;
    std::list<Choice> m_sorted_choices;
    std::list<Choice>::iterator m_choice_iterator;
  };
}


#endif

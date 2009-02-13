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
}

#endif

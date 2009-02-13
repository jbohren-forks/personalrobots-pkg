/**
 * @author Conor McGann
 */

#include "topological_map.hh"

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
    
}

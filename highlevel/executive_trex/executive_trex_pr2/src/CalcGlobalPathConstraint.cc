#include "CalcGlobalPathConstraint.hh"
#include "map_helpers.h"
#include "WavefrontPlanner.hh"
#include "Logger.hh"

namespace TREX {
  CalcGlobalPathConstraint::CalcGlobalPathConstraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						    const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_logger = TREX::Logger::request();
  }
  CalcGlobalPathConstraint::~CalcGlobalPathConstraint() {
    m_logger->release();
  }
  
  void CalcGlobalPathConstraint::handleExecute() {
    static unsigned int sl_cycles = 0;
    sl_cycles++;

    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());
    
    if(!m_variables[LOCAL_X]->lastDomain().isSingleton() ||
       !m_variables[LOCAL_Y]->lastDomain().isSingleton() ||
       !m_variables[GOAL_X]->lastDomain().isSingleton()  ||
       !m_variables[GOAL_Y]->lastDomain().isSingleton()) {
      return;
    }

    debugMsg("CalcGlobalPathConstraint:handleExecute", "Goal x " << m_variables[GOAL_X]->lastDomain().getSingletonValue() 
	      << " y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue());

    bool result = WavefrontPlanner::Instance()->GenerateGlobalPlan(m_variables[LOCAL_X]->lastDomain().getSingletonValue(),
								   m_variables[LOCAL_Y]->lastDomain().getSingletonValue(),
								   m_variables[GOAL_X]->lastDomain().getSingletonValue(),  
								   m_variables[GOAL_Y]->lastDomain().getSingletonValue()) >= 0 ;
    boolDom.set(result);

    FILE* log = m_logger->getFile();
    if (log) {
      fprintf(log, "\t<CalcGlobalPathConstraint time=\"%u\" value=\"%d\"/>\n", sl_cycles, result);
    }
  }
}

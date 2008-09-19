/**
 * @brief Collection of various components to use
 */
#include "ExecDefs.hh"
#include "OpenConditionManager.hh"
#include "FlawFilter.hh"
#include <list>

using namespace EUROPA::SOLVERS;

namespace TREX {
  class CostEstimator;
  class FloorFunction: public Constraint{
  public:
    FloorFunction(const LabelStr& name,
		  const LabelStr& propagatorName,
		  const ConstraintEngineId& constraintEngine,
		  const std::vector<ConstrainedVariableId>& variables);

  private:
    void handleExecute();
    AbstractDomain& m_target;
    const AbstractDomain& m_source;
  };






}

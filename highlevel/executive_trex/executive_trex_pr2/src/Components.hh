/**
 * @brief Collection of various components to use
 */
#include "Constraint.hh"

using namespace EUROPA;

namespace TREX {

  void signalHandler(int signalNo);

  void initROSExecutive(bool playback);

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

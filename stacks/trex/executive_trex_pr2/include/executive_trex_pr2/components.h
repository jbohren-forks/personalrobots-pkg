/**
 * @brief Collection of various components to use
 */
#include "Constraint.hh"
#include "Object.hh"
#include "AbstractDomain.hh"
#include "ConstrainedVariable.hh"
#include <string>
#include <vector>

using namespace EUROPA;

namespace TREX {

  void signalHandler(int signalNo);

  void initROSExecutive(bool playback);

  bool allSingletons(const std::vector<ConstrainedVariableId>& variables, const std::string& variables_to_check);

  class CostEstimator;
}

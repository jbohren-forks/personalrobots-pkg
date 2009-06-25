/**
 * @brief Collection of various components to use
 */
#include "Constraint.hh"
#include "Assembly.hh"
#include "Object.hh"
#include "AbstractDomain.hh"
#include "ConstrainedVariable.hh"
#include <string>
#include <vector>

using namespace EUROPA;

namespace TREX {
  typedef void (*SchemaFunction)(bool, const Assembly&);

  extern std::vector<SchemaFunction> _g_ros_schemas;

#define REGISTER_SCHEMA(type)						\
  void _do_not_use_schema##_##type(bool playback, const Assembly &a) {	\
    type(playback, a); }						\
  class _do_not_use_AutoGenClass##_##type {				\
  public:								\
    _do_not_use_AutoGenClass##_##type() {				\
      _g_ros_schemas.push_back(&(_do_not_use_schema##_##type));		\
    }									\
    static _do_not_use_AutoGenClass##_##type _instance;			\
  };									\
  _do_not_use_AutoGenClass##_##type					\
    _do_not_use_AutoGenClass##_##type::_instance;


  void signalHandler(int signalNo);

  void initROSExecutive(bool playback);

  bool allSingletons(const std::vector<ConstrainedVariableId>& variables, const std::string& variables_to_check);

  class CostEstimator;

  /**
   * @brief A base class constraint for posting an equality relation across matching
   * paramaters in a pair of tokens.
   * @note This could go back to the EUROPA constraint engine
   */
  class ParamEqConstraint: public Constraint{
  public:
    ParamEqConstraint(const LabelStr& name,
		       const LabelStr& propagatorName,
		       const ConstraintEngineId& constraintEngine,
		      const std::vector<ConstrainedVariableId>& variables,
		      const char* param_names);
  private:
    /**
     * @brief Takes a ':' delimited list of param names to use in applying the quality relation
     */
    std::vector<ConstrainedVariableId> makeNewScope(const char* param_names, const std::vector<ConstrainedVariableId>& variables);
    EntityId parentOf(const ConstrainedVariableId& var);
    ConstrainedVariableId getVariableByName(const ConstrainedVariableId& var, const LabelStr& param_name);
    void handleExecute();
    const TokenId _target_token;
    const TokenId _source_token;
  };
}

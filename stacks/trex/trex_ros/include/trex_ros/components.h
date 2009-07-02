/**
 * @brief Collection of various components to use
 */
#include "ros/ros.h"

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
  typedef void (*FactoryFunction)(bool);

  extern std::vector<SchemaFunction> _g_ros_schemas;
  extern std::vector<FactoryFunction> _g_ros_factories;

  template <SchemaFunction schema_function>
  class RegisterSchema {
    public:
      RegisterSchema() {
	// Make sure this schema has not been registered
	for (unsigned int i = 0; i < _g_ros_schemas.size(); i++) {
	  if(_g_ros_schemas[i] == schema_function) {
	    ROS_ERROR("Attempting to register the same schema multiple times!");
	    return;
	  }
	}

	_g_ros_schemas.push_back(schema_function);
      }
  };

  template <FactoryFunction factory_function>
  class RegisterFactory {
    public:
      RegisterFactory() {
	// Make sure this schema has not been registered
	for (unsigned int i = 0; i < _g_ros_factories.size(); i++) {
	  if(_g_ros_factories[i] == factory_function) {
	    ROS_ERROR("Attempting to register the same factory multiple times!");
	    return;
	  }
	}

	_g_ros_factories.push_back(factory_function);
      }
  };

#define REGISTER_SCHEMA(schema_function)			\
  TREX::RegisterSchema< schema_function > schema_register;
  
#define REGISTER_FACTORY(factory_function)					\
  TREX::RegisterFactory< factory_function > factory_register;

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

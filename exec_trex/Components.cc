#include "Components.hh"
#include "GoalManager.hh"
#include "PlanDatabase.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "TemporalAdvisor.hh"
#include "DbCore.hh"
#include "Constraints.hh"
#include "Timeline.hh"
#include "Agent.hh"
#include "CalcCommandConstraint.hh"
#include "CalcGlobalPathConstraint.hh"
#include "CalcArmInverseKinematicsConstraint.hh"
#include "CalcInterpolatedEndEffectorPosConstraint.hh"
#include "CalcGraspPositionConstraint.hh"
#include "CalcAngleDiffConstraint.hh"
#include "CalcCommandConstraintPlayback.hh"
#include "CalcGlobalPathConstraintPlayback.hh"
#include "CalcDistanceConstraint.hh"
#include "OrienteeringSolver.hh"

#include <math.h>

namespace TREX{

  class ROSSchema: public Assembly::Schema {
  public:
    ROSSchema(bool playback):m_playback(playback){}

    void registerComponents(const Assembly& assembly){
      Assembly::Schema::registerComponents(assembly);
      ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();

      // Constraint Registration
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::SubsetOfConstraint, "in", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::CalcDistanceConstraint, "calcDistance", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), FloorFunction, "calcFloor", "Default");

      if (m_playback) {
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcCommandConstraintPlayback, "calcCommand", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGlobalPathConstraintPlayback, "calcGlobalPath", "Default");
      } else {
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcCommandConstraint, "calcCommand", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGlobalPathConstraint, "calcGlobalPath", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcArmInverseKinematicsConstraint, "calcArmInverseKinematics", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGraspPositionConstraint, "calcGraspPosition", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcInterpolatedEndEffectorPosConstraint, "calcInterpolatedEndEffectorPos", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcAngleDiffConstraint, "calcAngleDiff", "Default");
      }
    }

  private:
    const bool m_playback;
  };

  ROSSchema* ROS_SCHEMA = NULL;

  void initROSExecutive(bool playback){
    initTREX();
    ROS_SCHEMA = new ROSSchema(playback);
  }

  FloorFunction::FloorFunction(const LabelStr& name,
		     const LabelStr& propagatorName,
		     const ConstraintEngineId& constraintEngine,
		     const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables),
      m_target(getCurrentDomain(m_variables[0])),
      m_source(getCurrentDomain(m_variables[1])){
    checkError(m_variables.size() == 2, "Exactly 2 parameters required. ");
  }

  void FloorFunction::handleExecute(){
    if(m_source.isSingleton()){
      int f = (int) m_source.getSingletonValue();
      m_target.set(f);
    }
  }


}

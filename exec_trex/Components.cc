#include "Components.hh"
#include "GoalManager.hh"
#include "PlanDatabase.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "TemporalAdvisor.hh"
#include "DbCore.hh"
#include "Constraints.hh"
#include "ConstraintLibrary.hh"
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


  void initROSExecutive(bool playback){

    initTREX();

    // Constraint Registration
    REGISTER_CONSTRAINT(TREX::SubsetOfConstraint, "in", "Default");
    REGISTER_CONSTRAINT(TREX::CalcDistanceConstraint, "calcDistance", "Default");
    REGISTER_CONSTRAINT(FloorFunction, "calcFloor", "Default");
    if (playback) {
      REGISTER_CONSTRAINT(CalcCommandConstraintPlayback, "calcCommand", "Default");
      REGISTER_CONSTRAINT(CalcGlobalPathConstraintPlayback, "calcGlobalPath", "Default");
    } else {
      REGISTER_CONSTRAINT(CalcCommandConstraint, "calcCommand", "Default");
      REGISTER_CONSTRAINT(CalcGlobalPathConstraint, "calcGlobalPath", "Default");
      REGISTER_CONSTRAINT(CalcArmInverseKinematicsConstraint, "calcArmInverseKinematics", "Default");
      REGISTER_CONSTRAINT(CalcGraspPositionConstraint, "calcGraspPosition", "Default");
      REGISTER_CONSTRAINT(CalcInterpolatedEndEffectorPosConstraint, "calcInterpolatedEndEffectorPos", "Default");
      REGISTER_CONSTRAINT(CalcAngleDiffConstraint, "calcAngleDiff", "Default");
    }
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

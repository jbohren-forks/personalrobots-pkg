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
#include "OrienteeringSolver.hh"


#include <gdk-pixbuf/gdk-pixbuf.h>
#include <libstandalone_drivers/plan.h>

#include <math.h>

namespace TREX{


  void initROSExecutive(bool playback){
    // Initialize glib
    g_type_init();

    initTREX();

    // Constraint Registration
    REGISTER_CONSTRAINT(SubsetOfConstraint, "in", "Default");
    REGISTER_CONSTRAINT(CalcDistanceConstraint, "calcDistance", "Default");
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

    // Solver Components
    REGISTER_FLAW_FILTER(TREX::GoalsOnlyFilter, GoalsOnly);
    REGISTER_FLAW_FILTER(TREX::NoGoalsFilter, NoGoals);
    REGISTER_FLAW_FILTER(TREX::DynamicGoalFilter, DynamicGoalFilter);
    REGISTER_FLAW_MANAGER(TREX::GoalManager, GoalManager);
    REGISTER_COMPONENT_FACTORY(TREX::EuclideanCostEstimator, EuclideanCostEstimator);
    REGISTER_COMPONENT_FACTORY(TREX::OrienteeringSolver, OrienteeringSolver); 
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

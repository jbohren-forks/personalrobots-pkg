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
#include "CalcAngleDiffConstraint.hh"
#include "CalcDistanceConstraint.hh"
#include "OrienteeringSolver.hh"

/*
#include "CalcCommandConstraint.hh"
#include "CalcGlobalPathConstraint.hh"
#include "CalcArmInverseKinematicsConstraint.hh"
#include "CalcInterpolatedEndEffectorPosConstraint.hh"
#include "CalcGraspPositionConstraint.hh"
#include "CalcCommandConstraintPlayback.hh"
#include "CalcGlobalPathConstraintPlayback.hh"
*/

#include <math.h>

namespace TREX{

  /**
   * @brief Handle cleanup on process termination signals.
   */
  void signalHandler(int signalNo){
    std::cout << "Handling signal..." << signalNo << std::endl;
    exit(0);
  }

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
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::NearestLocation, "nearestReachableLocation", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcAngleDiffConstraint, "calcAngleDiff", "Default");

      /*
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcArmInverseKinematicsConstraint, "calcArmInverseKinematics", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGraspPositionConstraint, "calcGraspPosition", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcInterpolatedEndEffectorPosConstraint, "calcInterpolatedEndEffectorPos", "Default");

      if (m_playback) {
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcCommandConstraintPlayback, "calcCommand", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGlobalPathConstraintPlayback, "calcGlobalPath", "Default");
      } else {
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcCommandConstraint, "calcCommand", "Default");
	REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcGlobalPathConstraint, "calcGlobalPath", "Default");
      }
      */
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

  NearestLocation::NearestLocation(const LabelStr& name,
				   const LabelStr& propagatorName,
				   const ConstraintEngineId& constraintEngine,
				   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables),
      m_x(getCurrentDomain(variables[0])),
      m_y(getCurrentDomain(variables[1])),
      m_location(static_cast<ObjectDomain&>(getCurrentDomain(variables[2]))){
    checkError(variables.size() == 3, "Invalid Arg Count: " << variables.size());
  }
  
  /**
   * Should wait till inputs are bound, then iterate over the locations and select the nearest one.
   */
  void NearestLocation::handleExecute() {
    static const LabelStr X("x");
    static const LabelStr Y("y");
    if(m_x.isSingleton() && m_y.isSingleton()){
      std::list<ObjectId> locations = m_location.makeObjectList();
      double minDistance = PLUS_INFINITY;
      ObjectId nearestLocation = locations.front();
      for(std::list<ObjectId>::const_iterator it = locations.begin(); it != locations.end(); ++it){
	ObjectId location = *it;
	ConstrainedVariableId x = location->getVariable(X);
	ConstrainedVariableId y = location->getVariable(Y);
	if(x.isId() && y.isId() && x->lastDomain().isSingleton() && y->lastDomain().isSingleton()){
	  double dx = m_x.getSingletonValue() - x->lastDomain().getSingletonValue();
	  double dy = m_y.getSingletonValue() - y->lastDomain().getSingletonValue();
	  double distance = sqrt(pow(dx, 2) + pow(dy, 2));
	  if(distance < minDistance){
	    nearestLocation = location;
	    minDistance = distance;
	  }
	}
      }

      m_location.set(nearestLocation);
    }
  }
}

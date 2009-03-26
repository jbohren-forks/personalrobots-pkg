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
#include "Utilities.hh"
#include <executive_trex_pr2/topological_map.h>
#include <executive_trex_pr2/door_domain_constraints.hh>

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

  class GetStateConstraint: public Constraint{
  public:
    GetStateConstraint(const LabelStr& name,
		       const LabelStr& propagatorName,
		       const ConstraintEngineId& constraintEngine,
		       const std::vector<ConstrainedVariableId>& variables);

  private:
    void handleExecute();
    std::string toString() const;

    const TokenId _target_token;
    const TokenId _source_token;
  };

  class AllBoundsSetConstraint: public Constraint{
  public:
    AllBoundsSetConstraint(const LabelStr& name,
			   const LabelStr& propagatorName,
			   const ConstraintEngineId& constraintEngine,
			   const std::vector<ConstrainedVariableId>& variables);

  private:
    void handleExecute();
    std::string toString() const;

    const TokenId _token;
  };

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


  class NearestLocation: public Constraint{
  public:
    NearestLocation(const LabelStr& name,
		    const LabelStr& propagatorName,
		    const ConstraintEngineId& constraintEngine,
		    const std::vector<ConstrainedVariableId>& variables);

  private:
    void handleExecute();
    const AbstractDomain& m_x;
    const AbstractDomain& m_y;
    ObjectDomain& m_location;
  };

  class RandomSelection: public Constraint{
  public:
    RandomSelection(const LabelStr& name,
		    const LabelStr& propagatorName,
		    const ConstraintEngineId& constraintEngine,
		    const std::vector<ConstrainedVariableId>& variables);

  private:
    void handleExecute();
    AbstractDomain& m_target;
  };

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
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::RandomSelection, "randomSelect", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), CalcAngleDiffConstraint, "calcAngleDiff", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::GetStateConstraint, "get_state", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), TREX::AllBoundsSetConstraint, "all_bounds_set", "Default");

      // Register topological map constraints
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
			  executive_trex_pr2::MapInitializeFromFileConstraint, "map_initialize_from_file", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(),
			  executive_trex_pr2::MapConnectorConstraint, "map_connector", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), 
			  executive_trex_pr2::MapGetRegionFromPositionConstraint, "map_get_region_from_position", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), 
			  executive_trex_pr2::MapIsDoorwayConstraint, "map_is_doorway", "Default");
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), 
			  executive_trex_pr2::MapGetDoorStateConstraint, "map_get_door_state", "Default");


      // Register functions for calculations in the door domain
      REGISTER_CONSTRAINT(constraintEngine->getCESchema(), 
			  executive_trex_pr2::GetRobotPoseForDoorConstraint, "door_get_robot_pose_for_door", "Default");

      // Register SOLVER components for topological map.
      EUROPA::SOLVERS::ComponentFactoryMgr* cfm = (EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr");
      REGISTER_FLAW_FILTER(cfm, executive_trex_pr2::MapConnectorFilter, MapConnectorFilter);
      REGISTER_FLAW_HANDLER(cfm, executive_trex_pr2::MapConnectorSelector, MapConnectorSelector);

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


  //*******************************************************************************************
  GetStateConstraint::GetStateConstraint(const LabelStr& name,
					 const LabelStr& propagatorName,
					 const ConstraintEngineId& constraintEngine,
					 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables), 
     _target_token(TREX::getParentToken(variables[0])), 
     _source_token(TREX::getParentToken(variables[1])){
    checkError(variables.size() == 2, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }
	 
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   * @todo Integarte proper code from Wim. Also assumes good data
   */
  void GetStateConstraint::handleExecute(){

    debugMsg("trex:propagation:get_state",  "BEFORE: " << toString());

    // Iterate over all parameters of the source token. Any with a name match will be intersected
    std::vector<ConstrainedVariableId>::const_iterator it = _source_token->parameters().begin();
    while (it != _source_token->parameters().end()){
      ConstrainedVariableId v_source = *it;
      ConstrainedVariableId v_target = _target_token->getVariable(v_source->getName());
      if(v_target.isId()){
	AbstractDomain& target_dom = getCurrentDomain(v_target);
	if(target_dom.intersect(v_source->lastDomain()) && target_dom.isEmpty()){
	  return;
	}
      }
      ++it;
    }

    debugMsg("trex:propagation:get_state",  "AFTER: " << toString());
  }

  std::string GetStateConstraint::toString() const {
    std::stringstream sstr;

    sstr << Entity::toString() << std::endl;

    std::vector<ConstrainedVariableId>::const_iterator it = _source_token->parameters().begin();
    unsigned int i = 0;
    while (it != _source_token->parameters().end()){
      ConstrainedVariableId v_source = *it;
      ConstrainedVariableId v_target = _target_token->getVariable(v_source->getName());
      if(v_target.isId())
	sstr << " ARG[" << i++ << "]:" << v_target->toLongString() << std::endl;
      ++it;
    }

    return sstr.str();
  }

  //*******************************************************************************************
  AllBoundsSetConstraint::AllBoundsSetConstraint(const LabelStr& name,
					 const LabelStr& propagatorName,
					 const ConstraintEngineId& constraintEngine,
					 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables), 
     _token(TREX::getParentToken(variables[0])){
    checkError(variables.size() == 1, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
  }
	 
  /**
   * Will fire if any bounds are not singletons
   */
  void AllBoundsSetConstraint::handleExecute(){
    debugMsg("trex:propagation:all_bounds_set",  "BEFORE: " << toString());

    // Iterate over all parameters of the source token. Any with a name match will be intersected
    for(std::vector<ConstrainedVariableId>::const_iterator it = _token->parameters().begin(); it != _token->parameters().end(); ++it){
      ConstrainedVariableId v = *it;
      if(!v->lastDomain().isSingleton()){
	debugMsg("trex:warning:propagation:all_bounds_set",  "Required parameter is unbound:" << v->toLongString());
	getCurrentDomain(v).empty();
	return;
      }
    }

    debugMsg("trex:propagation:all_bounds_set",  "AFTER: " << toString());
  }

  std::string AllBoundsSetConstraint::toString() const {
    std::stringstream sstr;

    sstr << Entity::toString() << std::endl;

    unsigned int i = 0;
    for(std::vector<ConstrainedVariableId>::const_iterator it = _token->parameters().begin(); it != _token->parameters().end(); ++it){
      ConstrainedVariableId v = *it;
      sstr << " ARG[" << i++ << "]:" << v->toLongString() << std::endl;
    }

    return sstr.str();
  }
  //*******************************************************************************************
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
    unsigned int iterations = 0;
    double minDistance = PLUS_INFINITY;

    debugMsg("trex:propagation:world_model:nearest_location",  "BEFORE: " << toString());

    if(m_x.isSingleton() && m_y.isSingleton()){
      std::list<ObjectId> locations = m_location.makeObjectList();      
      ObjectId nearestLocation = locations.front();
      for(std::list<ObjectId>::const_iterator it = locations.begin(); it != locations.end(); ++it){
	iterations++;
	ObjectId location = *it;
	ConstrainedVariableId x = location->getVariables()[0];
	ConstrainedVariableId y = location->getVariables()[1];
	checkError(x.isId(), "No variable for x");
	checkError(y.isId(), "No variable for y");
	checkError(x->lastDomain().isSingleton(), "Object variable for x should be bound but is not. " << x->toString());
	checkError(y->lastDomain().isSingleton(), "Object variable for y should be bound but is not. " << y->toString());
	double dx = m_x.getSingletonValue() - x->lastDomain().getSingletonValue();
	double dy = m_y.getSingletonValue() - y->lastDomain().getSingletonValue();
	double distance = sqrt(pow(dx, 2) + pow(dy, 2));

	// Should we promote?
	if(distance < minDistance){
	  nearestLocation = location;
	  minDistance = distance;
	}
      }

      m_location.set(nearestLocation);
    }

    debugMsg("trex:propagation:world_model:nearest_location",  "AFTER: " << toString() 
	      <<  std::endl << std::endl << "After " << iterations << " iterations, found a location within " << minDistance << " meters.");
  }


  RandomSelection::RandomSelection(const LabelStr& name,
				   const LabelStr& propagatorName,
				   const ConstraintEngineId& constraintEngine,
				   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables),
      m_target(getCurrentDomain(variables[0])){

    static bool initialized(false);
    checkError(variables.size() == 1, "Invalid Arg Count: " << variables.size());

    // Initialize random seed
    if(!initialized){
      srand(0);
      initialized = true;
    }
  }
  
  /**
   * Randomly choose a value from the propagated domain
   */
  void RandomSelection::handleExecute() {

    debugMsg("trex:propagation:random_select",  "BEFORE: " << toString());

    if(m_target.isEnumerated()){
      std::list<double> values;
      m_target.getValues(values);
      int selectionId = rand() % values.size();
      std::list<double>::const_iterator it = values.begin();
      while(selectionId > 0 && it != values.end()){
	selectionId--;
	++it;
      }
      m_target.set(*it);
    }
    else if(m_target.isFinite()){
      unsigned int delta = (unsigned int) (m_target.getUpperBound() - m_target.getLowerBound());
      int selectionId = rand() % delta;
      m_target.set(selectionId + m_target.getLowerBound());
    }
    else {// Do something
      m_target.set(m_target.getLowerBound());
    }

    debugMsg("trex:propagation:random_select",  "AFTER: " << toString());
  }
}

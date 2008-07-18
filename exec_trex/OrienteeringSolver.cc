
#include "OrienteeringSolver.hh"
#include "Token.hh"

namespace TREX {
  DynamicGoalFilter::DynamicGoalFilter(const TiXmlElement& configData): FlawFilter(configData, true) {}

  bool DynamicGoalFilter::test(const EntityId& entity){
    checkError(TokenId::convertable(entity), "Invalid configuration for " << entity->toString());

    TokenId token(entity);
    return !OrienteeringSolver::isGlobalNextGoal(token);
  }



  std::vector<OrienteeringSolverId> OrienteeringSolver::s_goalSolvers;

  bool OrienteeringSolver::isGlobalNextGoal(TokenId token) {
    for (std::vector<OrienteeringSolverId>::iterator it = s_goalSolvers.begin(); 
	 it != s_goalSolvers.end(); it++) {
      if ((*it)->isNextGoal(token)) {
	return true;
      }
    }
    return false;
  }

  bool OrienteeringSolver::isNextGoal(TokenId token) {
    return m_goalManager->isNextToken(token);
  }


  OrienteeringSolver::OrienteeringSolver(const TiXmlElement& cfgXml) 
    : FlawManagerSolver(cfgXml), m_goalManager(GoalManagerId::noId()), m_stepCount(0) {
    s_goalSolvers.push_back(this->getId());
  }
  
  OrienteeringSolver::~OrienteeringSolver() {
    for (std::vector<OrienteeringSolverId>::iterator it = s_goalSolvers.begin(); 
	 it != s_goalSolvers.end(); it++) {
      if ((OrienteeringSolver*)(*it) == this) {
	s_goalSolvers.erase(it);
	break;
      }
    }
  }
  
  void OrienteeringSolver::init(PlanDatabaseId db, TiXmlElement* cfgXml) {
    initDbListener(db, cfgXml);
    initFlawManagers(db, cfgXml);
    assertTrue(getFlawManagerCount() == 1, 
	       "You must have one and only one GoalManager per solver, and nothing else.");
    assertTrue((GoalManager*)getFlawManager(0),
	       "You must have one and only one GoalManager per solver, and nothing else.");
    m_goalManager = ((GoalManager*)getFlawManager(0))->getId();
  }
  
  bool OrienteeringSolver::isExhausted() {
    return false;
  }
  
  void OrienteeringSolver::step() { 
    if (!noMoreFlaws()) {
      m_stepCount++;
      m_goalManager->step();
    }
  }
  
  unsigned int OrienteeringSolver::getDepth() {
    return m_stepCount;
  }
  
  unsigned int OrienteeringSolver::getStepCount() {
    return m_stepCount;
  }
  
  
  bool OrienteeringSolver::noMoreFlaws() {
    return m_goalManager->noMoreFlaws();
  }
  
  
  void OrienteeringSolver::clear() {
  }
  
  
  void OrienteeringSolver::reset() {
  }
  
  
}

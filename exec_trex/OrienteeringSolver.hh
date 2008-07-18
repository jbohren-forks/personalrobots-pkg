#ifndef H_ORIENTEERINGSOLVER
#define H_ORIENTEERINGSOLVER

#include "DbSolver.hh"
#include "GoalManager.hh"

namespace TREX {
  class DynamicGoalFilter : public FlawFilter {
  public:
    DynamicGoalFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  };

  class OrienteeringSolver;
  typedef Id<OrienteeringSolver> OrienteeringSolverId;

  /**
   * @brief A Steepest Ascent Hill Climbing Algorithm for Orienteering Problems.
   */
  class OrienteeringSolver : public FlawManagerSolver {
  public:
    /**
     * @brief Creates the OrienteringSolver.
     */
    OrienteeringSolver(const TiXmlElement& cfgXml);
    /**
     * @brief Destroys the OrienteringSolver.
     */
    ~OrienteeringSolver();
    /**
     * @brief Inits the solver.
     */
    void init(PlanDatabaseId db, TiXmlElement* cfgXml);
    /**
     * @brief Tests if the solver has exhausted its search space.
     */
    bool isExhausted();
    /**
     * @brief Steps the solver.
     */
    void step();
    /**
     * @brief Gets the depth of the solver.
     */
    unsigned int getDepth();
    /**
     * @brief Gets the step count of the solver.
     */
    unsigned int getStepCount();
    /**
     * @brief Tests if there are no more flaws for the solver.
     */
    bool noMoreFlaws();
    /**
     * @brief Clears current decisions on the stack without any modifications to the plan.
     */
    void clear();
    /**
     * @brief Retracts all decisions stored in the internal decision stack.
     * @note Will not force propagation.
     */
    void reset();
    /**
     * @brief Tests all orientering solvers to see if the token is the next goal.
     */
    static bool isGlobalNextGoal(TokenId token);
    /**
     * @brief Tests is the token is the next goal.
     */
    bool isNextGoal(TokenId token);
  private:
    GoalManagerId m_goalManager; /*! The goal manager. */
    static std::vector<OrienteeringSolverId> s_goalSolvers;  /*! Static list of solvers. */
    unsigned int m_stepCount; /*! Counts steps. */
  };
}



#endif


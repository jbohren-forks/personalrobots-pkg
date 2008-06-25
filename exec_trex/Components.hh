/**
 * @brief Collection of various components to use
 */
#include "ExecDefs.hh"
#include "OpenConditionManager.hh"
#include "FlawFilter.hh"
#include <list>

using namespace EUROPA::SOLVERS;

namespace TREX {

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


  class GoalsOnlyFilter: public FlawFilter {
  public:
    GoalsOnlyFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  };

  class NoGoalsFilter: public FlawFilter {
  public:
    NoGoalsFilter(const TiXmlElement& configData);
    bool test(const EntityId& entity);
  };

  struct Position {
    double x;
    double y;
  };

  /**
   * @brief Goal Manager to manage goal flaw selection. Will involve solving an orienteering problem
   * over the relaxed problem to provide a good ordering. We can imagine this manager maintaining
   * a priority q and releasing goals 1 at a time.
   */
  class GoalManager: public OpenConditionManager {
  public:

    /**
     * @brief Solution is a sequence of tokens. None can be rejected. Each has a 2d position.
     */
    typedef std::list<TokenId> SOLUTION;

    /**
     * @brief Uses standard constructor
     */
    GoalManager(const TiXmlElement& configData);

    /**
     * @brief Handles the iteration. Will always impose a limit of the next goal
     */
    IteratorId createIterator();

    // Assumptions about the fields in a goal
    DECLARE_STATIC_CLASS_CONST(LabelStr, X, "x");
    DECLARE_STATIC_CLASS_CONST(LabelStr, Y, "y");
    DECLARE_STATIC_CLASS_CONST(LabelStr, PRIORITY, "Priority");

    // Parameters used for configuration
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_POSITION_SOURCE, "positionSource");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAP_SOURCE, "mapSource");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAX_ITERATIONS, "maxIterations");
    DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_PLATEAU, "plateau");

  private:   
    /**
     * @brief A Trivial iterator - one goal only. Used to return a goal to the solver
     */
    class Iterator : public FlawIterator {
    public:
      Iterator(GoalManager& manager, const TokenId& nextGoal);

    private:
      const EntityId nextCandidate();
      TokenId m_nextGoal;
    };

    /**
     * @brief Used to synch mark current goal value as dirty
     * @see OpenConditionManager::addFlaw
     */
    void addFlaw(const TokenId& token);

    /**
     * @brief Used to synch mark current goal value as dirty
     * @see OpenConditionManager::removeFlaw
     */
    void removeFlaw(const TokenId& token);

    /**
     * @brief Used to synch mark current goal value as dirty
     */
    void handleInitialize();

    /**
     * @brief Generate an initial solution. May not be feasible.
     */
    void generateInitialSolution();

    /**
     * @brief Search for best solution
     */
    void search();

    /**
     * @brief Helper method
     */
    int getPriority(const TokenId& token);

    /**
     * @brief Evaluate the solution s. This will compute both the cost and the utility.
     * @return true if feasible, false if infeasible
     */
    bool evaluate(const SOLUTION& s, double& cost, double& utility);

    /**
     * @brief Compute a neighboring solution for s
     */
    void selectNeighbor(GoalManager::SOLUTION& s, TokenId& delta);

    /**
     * @brief Set initial conditions in terms of position, time and energy
     */
    void setInitialConditions();

    /**
     * @brief Get a distance estimate between points.
     */
    double computeDistance(const Position& p1, const Position& p2);

    /**
     * @brief Accessor to get a Position. For convenience
     */
    static Position getPosition(const TokenId& token);

    /**
     * @brief Accessor to get current position (at current tick)
     */
    Position getCurrentPosition() const;

    /**
     * @brief Accessor for robot speed
     */
    double getSpeed() const;

    /**
     * @brief Comparator
     */
    int compare(const SOLUTION& s1, const SOLUTION& s2);

    void insert(SOLUTION&s, const TokenId& t, unsigned int pos);
    void swap(SOLUTION& s, unsigned int a, unsigned int b);
    void remove(SOLUTION& s, const TokenId& t);
    void update(SOLUTION& s, TokenId& delta, const SOLUTION& c, TokenId t);

    std::string toString(const SOLUTION& s);

    // Configuration derived members
    unsigned int m_maxIterations;
    unsigned int m_plateau;
    LabelStr m_positionSourceCfg;
    TimelineId m_positionSource;

    unsigned int m_cycleCount;
    unsigned int m_lastCycle;
    SOLUTION m_currentSolution;
    TokenSet m_ommissions;

    /*!< INITIAL CONDITIONS */
    int m_startTime;
    double m_timeBudget;

    // Integration with wavefront planner
    //plan_t* wv_plan;

    static const int WORSE = -1;
    static const int EQUAL = 0;
    static const int BETTER = 1;
  };
}

/**
 * @author Conor McGann
 * @brief Implements bindings for constraints and other queries for accessing the topological map
 */

#ifndef _TOPOLOGICAL_MAP_H_
#define _TOPOLOGICAL_MAP_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "IntervalDomain.hh"
#include "IntervalIntDomain.hh"
#include "BoolDomain.hh"
#include "Logger.hh"
#include "FlawFilter.hh"
#include "UnboundVariableDecisionPoint.hh"

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {
  
  /**
   * @brief A relation: given a connector, bind the x, y the values. Given x, and y, bind the connector
   */
  class map_connector_constraint : public Constraint {
  public:
    
    map_connector_constraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~map_connector_constraint();
    
    void handleExecute();
    
  private:
    IntervalIntDomain& m_connector;
    IntervalDomain& m_x;
    IntervalDomain& m_y;
    IntervalDomain& m_th;
  };

  /**
   * @brief A function: given an x,y position, bind a region.
   */
  class map_get_region_from_position_constraint : public Constraint {
  public:
    
    map_get_region_from_position_constraint(const LabelStr& name,
					    const LabelStr& propagatorName,
					    const ConstraintEngineId& constraintEngine,
					    const std::vector<ConstrainedVariableId>& variables);
    
    ~map_get_region_from_position_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A relation: constrain two connection variables so that they share a common region. Propagates when one of 3 is bound.
   */
  class map_connected_constraint : public Constraint {
  public:
    
    map_connected_constraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~map_connected_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A function to query if a region is a doorway
   */
  class map_is_doorway_constraint : public Constraint {
  public:
    
    map_is_doorway_constraint(const LabelStr& name,
			      const LabelStr& propagatorName,
			      const ConstraintEngineId& constraintEngine,
			      const std::vector<ConstrainedVariableId>& variables);
    
    ~map_is_doorway_constraint();
    
    void handleExecute();
    
  private:
    
  };

  /**
   * @brief A filter to exclude variable binding decisions unless they are on a parameter variable
   * of a token for has all required variables, and the source and final destination are already singletons
   */
  class map_connector_filter: public SOLVERS::FlawFilter {
  public:
    map_connector_filter(const TiXmlElement& configData);

    bool test(const EntityId& entity);

  private:
    const LabelStr m_source;
    const LabelStr m_final;
    const LabelStr m_target;
  };


  /**
   * @brief Implements a sort based on minimizing g_cost + h_cost for target selection
   */
  class map_connector_selector: public SOLVERS::UnboundVariableDecisionPoint {
  public:
    map_connector_selector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation = "unknown");
    bool hasNext() const;
    double getNext();

  private:
    class Choice {
    public:
      Choice():id(0), cost(0){}
      unsigned int id;
      double cost;  
      bool operator<(const map_connector_selector::Choice& rhs) const {return cost < rhs.cost;}
    };

    double g_cost(unsigned int from, unsigned int to) const;
    double h_cost(unsigned int from, unsigned int to) const;

    const LabelStr m_source;
    const LabelStr m_final;
    std::list<Choice> m_sorted_choices;
    std::list<Choice>::iterator m_choice_iterator;
  };


  /**
   * @brief A TopologicalMapAccessor is used to access the data of a topological map. This accessor will
   * be used by the constraints and flaw filter. The accessor is a singleton.
   */
  class TopologicalMapAccessor {
  public:

    /**
     * @brief Singleton accessor
     * @return NULL if no instance created yet.
     */
    static TopologicalMapAccessor* instance();

    /**
     * @brief Get a region for a point
     * @retun 0 if no region found, otherwise the region for the given point
     */
    virtual unsigned int get_region(double x, double y);

    /**
     * @brief Get a connector for a point
     * @retun 0 if no connector found, otherwise the connector for the given point
     */
    virtual unsigned int get_connector(double x, double y);

    /**
     * @brief Get position details for a connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool get_connector_position(unsigned int connector_id, double& x, double& y, double& theta);

    /**
     * @brief Get the connectors of a particular region
     * @return true if the given region id was valid, otherwose false.
     */
    virtual bool get_region_connectors(unsigned int region_id, std::list<unsigned int>& connectors);

    /**
     * @brief Get the regions of a particular connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool get_connector_regions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b);

    /**
     * @brief Test if a given region is a doorway
     * @return true if a doorway, otherwise false. A 0 id region is not a doorway.
     */
    virtual bool is_doorway(unsigned int region_id);

    /**
     * @brief Get the cost to go from a given 2D point to a connector. The point must be in a region
     * accessible by the connector
     * @return PLUS_INFINITY if not reachable (e.g. not in the same region or a bad id. Otherwise the cost to get there.
     */
    virtual double get_g_cost(double from_x, double from_y, unsigned int connector_id);

    /**
     * @brief Get the cost to go from a given connector, to a final destination given by a point in space
     * @return PLUS_INFINITY if not reachable (e.g. not in the same region or a bad id. Otherwise the cost to get there.
     */
    virtual double get_h_cost(unsigned int connector_id, double to_x, double to_y);

    virtual ~TopologicalMapAccessor();

  protected:
    TopologicalMapAccessor();
  };
}


#endif

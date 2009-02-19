/**
 * @author Conor McGann
 * @brief Implements bindings for constraints and other queries for accessing the topological map
 */

#ifndef EXECUTIVE_TREX_PR2_TOPOLOGICAL_MAP_H_
#define EXECUTIVE_TREX_PR2_TOPOLOGICAL_MAP_H_

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
#include <topological_map/topological_map.h>

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {
  
  /**
   * @brief A relation: given a connector, bind the x, y the values. Given x, and y, bind the connector
   */
  class MapConnectorConstraint : public Constraint {
  public:
    
    MapConnectorConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _connector;
    IntervalDomain& _x;
    IntervalDomain& _y;
  };

  /**
   * @brief A function: given an x,y position, bind a region.
   */
  class MapGetRegionFromPositionConstraint : public Constraint {
  public:
    
    MapGetRegionFromPositionConstraint(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _region;
    IntervalDomain& _x;
    IntervalDomain& _y;
  };

  /**
   * @brief A relation: constrain two connection variables so that they share a common region. Propagates when one of 3 is bound.
   */
  class MapConnectedConstraint : public Constraint {
  public:
    
    MapConnectedConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    virtual void handleExecute();
    
  private:
    
  };

  /**
   * @brief A function to query if a region is a doorway
   */
  class MapIsDoorwayConstraint : public Constraint {
  public:
    
    MapIsDoorwayConstraint(const LabelStr& name,
			      const LabelStr& propagatorName,
			      const ConstraintEngineId& constraintEngine,
			      const std::vector<ConstrainedVariableId>& variables);
    
    virtual void handleExecute();
    
  private:
    
  };

  /**
   * @brief A filter to exclude variable binding decisions unless they are on a parameter variable
   * of a token for has all required variables, and the source and final destination are already singletons
   */
  class MapConnectorFilter: public SOLVERS::FlawFilter {
  public:
    MapConnectorFilter(const TiXmlElement& configData);

    virtual bool test(const EntityId& entity);

  private:
    const LabelStr _source;
    const LabelStr _final;
    const LabelStr _target;
  };


  /**
   * @brief Implements a sort based on minimizing g_cost + h_cost for target selection
   */
  class MapConnectorSelector: public SOLVERS::UnboundVariableDecisionPoint {
  public:
    MapConnectorSelector(const DbClientId& client, const ConstrainedVariableId& flawedVariable, const TiXmlElement& configData, const LabelStr& explanation = "unknown");
    bool hasNext() const;
    double getNext();

  private:
    class Choice {
    public:
      Choice():id(0), cost(0){}
      unsigned int id;
      double cost;  
      bool operator<(const MapConnectorSelector::Choice& rhs) const {return cost < rhs.cost;}
    };

    double gCost(unsigned int from, unsigned int to) const;
    double hCost(unsigned int from, unsigned int to) const;

    const LabelStr _source;
    const LabelStr _final;
    std::list<Choice> _sorted_choices;
    std::list<Choice>::iterator _choice_iterator;
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
    virtual unsigned int getRegion(double x, double y) = 0;

    /**
     * @brief Get a connector for a point
     * @retun 0 if no connector found, otherwise the connector for the given point
     */
    virtual unsigned int getConnector(double x, double y) = 0;

    /**
     * @brief Get position details for a connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool getConnectorPosition(unsigned int connector_id, double& x, double& y) = 0;

    /**
     * @brief Get the connectors of a particular region
     * @return true if the given region id was valid, otherwose false.
     */
    virtual bool getRegionConnectors(unsigned int region_id, std::vector<unsigned int>& connectors) = 0;

    /**
     * @brief Get the regions of a particular connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool getConnectorRegions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b) = 0;

    /**
     * @brief Test if a given region is a doorway
     * @param result set to true if a doorway, otherwise false.
     * @return true if it is a valid region, otherwise false
     */
    virtual bool isDoorway(unsigned int region_id, bool& result) = 0;

    /**
     * @brief Test if a point is in collision
     * @return true if it is in an obstacle
     */
    virtual bool isObstacle(double x, double y) = 0;

    /**
     * @brief Get the cost to go from a given 2D point to a connector. The point must be in a region
     * accessible by the connector
     * @return PLUS_INFINITY if not reachable (e.g. not in the same region or a bad id. Otherwise the cost to get there.
     */
    virtual double gCost(double from_x, double from_y, unsigned int connector_id) = 0;

    /**
     * @brief Get the cost to go from a given connector, to a final destination given by a point in space
     * @return PLUS_INFINITY if not reachable (e.g. not in the same region or a bad id. Otherwise the cost to get there.
     */
    virtual double hCost(unsigned int connector_id, double to_x, double to_y) = 0;

    virtual ~TopologicalMapAccessor();

    double getResolution() const{return _resolution;}

  protected:
    TopologicalMapAccessor(double resolution);

  private:
    static TopologicalMapAccessor* _singleton;
    const double _resolution;
  };

  /**
   * @brief This class implements and adapter to the topological map derived from a bottleneck analysis of an occupancy grid
   */
  class TopologicalMapAdapter: public TopologicalMapAccessor {
  public:

    TopologicalMapAdapter(const topological_map::OccupancyGrid& grid, double resolution);

    virtual ~TopologicalMapAdapter();

    virtual unsigned int getRegion(double x, double y);

    virtual unsigned int getConnector(double x, double y);

    virtual bool getConnectorPosition(unsigned int connector_id, double& x, double& y);

    virtual bool getRegionConnectors(unsigned int region_id, std::vector<unsigned int>& connectors);

    virtual bool getConnectorRegions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b);

    virtual bool isDoorway(unsigned int region_id, bool& result);

    virtual bool isObstacle(double x, double y);

    virtual double gCost(double from_x, double from_y, unsigned int connector_id);

    virtual double hCost(unsigned int connector_id, double to_x, double to_y);

  private:
    topological_map::TopologicalMapPtr _map;
  };
}


#endif

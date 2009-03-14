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
  
  class TopologicalMapAdapter;


  /**
   * @brief Simple container class for a connection cost pair
   */
  class ConnectionCostPair {
  public:
    ConnectionCostPair():id(0), cost(0){}
    ConnectionCostPair(unsigned int id_, double cost_):id(id_), cost(cost_) {}

    unsigned int id;
    double cost;  
    bool operator<(const ConnectionCostPair& rhs) const {return cost < rhs.cost;}
  };

  /**
   * @brief A procedure to load a topological map from a file
   */
  class MapInitializeFromFileConstraint  : public Constraint {
  public:
    
    MapInitializeFromFileConstraint(const LabelStr& name,
				    const LabelStr& propagatorName,
				    const ConstraintEngineId& constraintEngine,
				    const std::vector<ConstrainedVariableId>& variables);
    
    virtual void handleExecute(){}

    virtual ~MapInitializeFromFileConstraint();

  private:

    TopologicalMapAdapter* _map;
  };

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
    BoolDomain& _result;
    IntervalIntDomain& _region;
  };

  /**
   * @brief A function: given 2 points in space, return a door that is in the region traversed by their connecting
   * line
   */
  class MapGetDoorFromPositionConstraint : public Constraint {
  public:
    
    MapGetDoorFromPositionConstraint(const LabelStr& name,
				     const LabelStr& propagatorName,
				     const ConstraintEngineId& constraintEngine,
				     const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _door;
    IntervalDomain& _x1;
    IntervalDomain& _y1;
    IntervalDomain& _x2;
    IntervalDomain& _y2;
  };

  /**
   * @brief A function: given a door id, obtain the door frame points from the map
   */
  class MapGetDoorDataConstraint : public Constraint {
  public:
    
    MapGetDoorDataConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    IntervalDomain& _x1;
    IntervalDomain& _y1;
    IntervalDomain& _x2;
    IntervalDomain& _y2;
    IntervalIntDomain& _door;
  };

  /**
   * @brief A function: given a door id, obtain the door handle information
   */
  class MapGetHandlePositionConstraint : public Constraint {
  public:
    
    MapGetHandlePositionConstraint(const LabelStr& name,
				   const LabelStr& propagatorName,
				   const ConstraintEngineId& constraintEngine,
				   const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    IntervalDomain& _x;
    IntervalDomain& _y;
    IntervalDomain& _z;
    IntervalIntDomain& _door;
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
    bool noGoodInput(const TokenId& parent_token, const LabelStr& var_name) const;

    const LabelStr _source_x;
    const LabelStr _source_y;
    const LabelStr _final_x;
    const LabelStr _final_y;
    const LabelStr _target_connector;
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
    std::string toString() const;

    const LabelStr _source_x;
    const LabelStr _source_y;
    const LabelStr _final_x;
    const LabelStr _final_y;
    const LabelStr _target_connector;
    std::list<ConnectionCostPair> _sorted_choices;
    std::list<ConnectionCostPair>::iterator _choice_iterator;
  };


  /**
   * @brief A TopologicalMapAdapter is used to access the data of a topological map. This accessor will
   * be used by the constraints and flaw filter. The accessor is a singleton.
   */
  class TopologicalMapAdapter {
  public:

    /**
     * @brief Singleton accessor
     * @return NULL if no instance created yet.
     */
    static TopologicalMapAdapter* instance();

    /**
     * @brief Constructor reading in from a serialized file of a prior map
     */
    TopologicalMapAdapter(std::istream& in);

    /**
     * @brief Constructor based on a grid
     */
    TopologicalMapAdapter(const topological_map::OccupancyGrid& grid, double resolution);

    /**
     * @brief Destructor is virtual for subclasses to over-ride
     */
    virtual ~TopologicalMapAdapter();

    /**
     * @brief Get a region for a point
     * @retun 0 if no region found, otherwise the region for the given point
     */
    virtual unsigned int getRegion(double x, double y);

    /**
     * @brief Get a region for a point
     * @retun Return the cells for a given region id
     */
    virtual topological_map::RegionPtr  getRegionCells(unsigned int region_id);

    /**
     * @brief Get a connector for a point
     * @retun 0 if no connector found, otherwise the connector for the given point
     */
    virtual unsigned int getConnector(double x, double y);

    /**
     * @brief Get position details for a connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool getConnectorPosition(unsigned int connector_id, double& x, double& y);

    /**
     * @brief Get the connectors of a particular region
     * @return true if the given region id was valid, otherwose false.
     */
    virtual bool getRegionConnectors(unsigned int region_id, std::vector<unsigned int>& connectors);

    /**
     * @brief Get the regions of a particular connector
     * @return true if the given connector id was valid, otherwise false.
     */
    virtual bool getConnectorRegions(unsigned int connector_id, unsigned int& region_a, unsigned int& region_b);

    /**
     * @brief Get the door id given a pair of points
     * @return 0 if no door found, otherwise the id for a door
     */
    virtual unsigned int getDoorFromPosition(double x1, double y1, double x2, double y2);

    /**
     * @brief Get the door position information (2 points at its base, given the id
     * @return true if the door id is valid, otherwise false. If a valid id, then it will fill out point data
     */
    virtual bool getDoorData(double& x1, double& y1, double& x2, double& y2, unsigned int door_id);

    /**
     * @brief Test if a given region is a doorway
     * @param result set to true if a doorway, otherwise false.
     * @return true if it is a valid region, otherwise false
     */
    virtual bool isDoorway(unsigned int region_id, bool& result);

    /**
     * @brief Test if a point is in collision
     * @return true if it is in an obstacle
     */
    virtual bool isObstacle(double x, double y);

    /**
     * @brief Get the cost to go from one 2D point to another.
     * @return PLUS_INFINITY if not reachable (e.g. not in the same region or a bad id. Otherwise the cost to get there.
     */
    virtual double cost(double from_x, double from_y, double to_x, double to_y);

    /**
     * @brief Get the to travel between a point and a connector
     * @return PLUS_INFINITY if not reachable or a bad connctor id
     */
    virtual double cost(double to_x, double to_y, unsigned int connector_id);

    /**
     * @broef Query for local connectors with costs to goal data as well
     */
    virtual void getLocalConnectionsForGoal(std::list<ConnectionCostPair>& results, double x0, double y0, double x1, double y1);

  private:
    static TopologicalMapAdapter* _singleton;

    // Helper method to visualize the graph as a post script file
    void toPostScriptFile();

    // Helper method to visualize as a ppm
    std::string toPPM();

    topological_map::TopologicalMapPtr _map;
  };
}


#endif

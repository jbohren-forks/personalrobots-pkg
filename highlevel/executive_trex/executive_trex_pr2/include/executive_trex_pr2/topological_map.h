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

    class Choice {
    public:
    Choice():id(0), cost(0){}
    Choice(unsigned int id_, double cost_):id(id_), cost(cost_) {}

      unsigned int id;
      double cost;  
      bool operator<(const MapConnectorSelector::Choice& rhs) const {return cost < rhs.cost;}
    };

    double gCost(unsigned int from, unsigned int to) const;
    double hCost(unsigned int from, unsigned int to) const;

    const LabelStr _source_x;
    const LabelStr _source_y;
    const LabelStr _final_x;
    const LabelStr _final_y;
    const LabelStr _target_connector;
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
     * @brief Destructor is virtual for subclasses to over-ride
     */
    virtual ~TopologicalMapAccessor();

    /**
     * @brief The number of meters per cell in the metric representation
     */
    double getResolution() const{return _resolution;}

    /**
     * @brief The number of columns
     */
    unsigned int getNumCols() const {return _num_cols;}

    /**
     * @brief The number of rows
     */
    unsigned int getNumRows() const {return _num_rows;}

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
     * @brief Get the door id given a pair of points
     * @return 0 if no door found, otherwise the id for a door
     */
    virtual unsigned int getDoorFromPosition(double x1, double y1, double x2, double y2) = 0;

    /**
     * @brief Get the door position information (2 points at its base, given the id
     * @return true if the door id is valid, otherwise false. If a valid id, then it will fill out point data
     */
    virtual bool getDoorData(double& x1, double& y1, double& x2, double& y2, unsigned int door_id) = 0;

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

  protected:
    TopologicalMapAccessor(double resolution, unsigned int cols, unsigned int rows);

  private:
    static TopologicalMapAccessor* _singleton;
    const double _resolution;
    const unsigned int _num_cols;
    const unsigned int _num_rows;
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

    virtual unsigned int getDoorFromPosition(double x1, double y1, double x2, double y2);

    virtual bool getDoorData(double& x1, double& y1, double& x2, double& y2, unsigned int door_id);

    virtual bool isDoorway(unsigned int region_id, bool& result);

    virtual bool isObstacle(double x, double y);

    virtual double gCost(double from_x, double from_y, unsigned int connector_id);

    virtual double hCost(unsigned int connector_id, double to_x, double to_y);

  private:
    // Helper method to visualize the grap as a post script file
    void toPostScriptFile();

    topological_map::TopologicalMapPtr _map;
  };
}


#endif

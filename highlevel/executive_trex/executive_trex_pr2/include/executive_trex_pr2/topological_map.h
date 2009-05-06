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
#include "Domains.hh"
#include "Logger.hh"
#include "FlawFilter.hh"
#include "UnboundVariableDecisionPoint.hh"
#include <topological_map/topological_map.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/Pose.h>

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
   * @brief Computes inputs for the next move
   */
  class MapGetNextMoveConstraint : public Constraint {
  public:
    
    MapGetNextMoveConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    virtual void handleExecute();
    
  private:
    AbstractDomain& _next_x;
    AbstractDomain& _next_y;
    AbstractDomain& _next_z;
    AbstractDomain& _next_qx;
    AbstractDomain& _next_qy;
    AbstractDomain& _next_qz;
    AbstractDomain& _next_qw;
    BoolDomain& _thru_doorway;
    AbstractDomain& _current_x;
    AbstractDomain& _current_y;
    AbstractDomain& _target_x;
    AbstractDomain& _target_y;
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
   * @brief A function: given an x,y position, bind a region.
   */
  class MapGetDoorwayFromPointsConstraint : public Constraint {
  public:
    
    MapGetDoorwayFromPointsConstraint(const LabelStr& name,
				      const LabelStr& propagatorName,
				      const ConstraintEngineId& constraintEngine,
				      const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _region;
    IntervalDomain& _x1;
    IntervalDomain& _y1;
    IntervalDomain& _x2;
    IntervalDomain& _y2;
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
   * @brief A function: given a door id, obtain all door data
   */
  class MapGetDoorStateConstraint : public Constraint {
  public:
    
    MapGetDoorStateConstraint(const LabelStr& name,
			      const LabelStr& propagatorName,
			      const ConstraintEngineId& constraintEngine,
			      const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    std::string toString() const;

    bool apply(double value, const char* param_name);

    TokenId _token_id;
    IntervalIntDomain& _door_id;
  };


  /**
   * @brief Updates the map to indicate the given doorway is blocked. This operation is monotonic. the doorway will never
   * be recognized as unblocked.
   */
  class MapNotifyDoorBlockedConstraint : public Constraint {
  public:
    
    MapNotifyDoorBlockedConstraint(const LabelStr& name,
				   const LabelStr& propagatorName,
				   const ConstraintEngineId& constraintEngine,
				   const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _doorway;
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
   * @brief A function: given an x,y position, find the nearest outlet.
   */
  class MapGetNearestOutletConstraint : public Constraint {
  public:
    
    MapGetNearestOutletConstraint(const LabelStr& name,
				  const LabelStr& propagatorName,
				  const ConstraintEngineId& constraintEngine,
				  const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _outlet;
    IntervalDomain& _x;
    IntervalDomain& _y;
  };



  /**
   * @brief A function: Get the outlet position and orientation.
   */
  class MapGetOutletStateConstraint : public Constraint {
  public:
    
    MapGetOutletStateConstraint(const LabelStr& name,
				const LabelStr& propagatorName,
				const ConstraintEngineId& constraintEngine,
				const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalDomain& _x;
    IntervalDomain& _y;
    IntervalDomain& _z;
    IntervalDomain& _qx;
    IntervalDomain& _qy;
    IntervalDomain& _qz;
    IntervalDomain& _qw;
    const IntervalIntDomain& _outlet;
  };

  /**
   * @brief Updates the map to indicate the given outlet is blocked. This operation is monotonic. the outlet will never
   * be recognized as unblocked.
   */
  class MapNotifyOutletBlockedConstraint : public Constraint {
  public:
    
    MapNotifyOutletBlockedConstraint(const LabelStr& name,
				     const LabelStr& propagatorName,
				     const ConstraintEngineId& constraintEngine,
				     const std::vector<ConstrainedVariableId>& variables);
    virtual void handleExecute();
    
  private:
    IntervalIntDomain& _outlet;
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
     * @brief Get costs
     */
    virtual void getConnectorCosts(double x0, double y0, double x1, double y1, std::vector< std::pair<topological_map::ConnectorId, double> >& results);

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
     * @brief Get the door position information (2 points at its base, given the id
     * @return true if the door id is a valid doorway, otherwise false. If a valid id, then it will the door message
     */
    virtual bool getDoorState(unsigned int doorway_id, robot_msgs::Door& door_state);

    /**
     * @brief Test if a given region is a doorway
     * @param result set to true if a doorway, otherwise false.
     * @return true if it is a valid region, otherwise false
     */
    virtual bool isDoorway(unsigned int region_id, bool& result);

    /**
     * @brief Door blocked
     */
    virtual void observeDoorBlocked(unsigned int door_id);

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
     * @brief Query for local connectors with costs to goal data as well
     */
    virtual void getLocalConnectionsForGoal(std::list<ConnectionCostPair>& results, double x0, double y0, double x1, double y1);

    /**
     * @brief Get the nearest outlet given a 2d point
     */
    virtual unsigned int getNearestOutlet(double x, double y);

    /**
     * @brief Query detailed outlet data. Might want to think about adding and OutletState msg
     */
    virtual void getOutletState(unsigned int outlet_id, robot_msgs::Pose& outlet_pose);

    /**
     * @brief Outlet blocked
     */
    virtual void observeOutletBlocked(unsigned int outlet_id);

    /**
     */
    virtual unsigned int getNearestDoorway(double x, double y);

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

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Contains implementation definitions for topological map
 * (not meant to be externally included)
 */

#ifndef TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_IMPL_H
#define TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_IMPL_H

#include "topological_map.h"
#include <boost/tuple/tuple.hpp>

namespace topological_map
{

using std::map;
using std::ostream;
using std::istream;
using boost::tuple;
using boost::shared_ptr;
using door_msgs::Door;
using ros::Time;

class RegionGraph;
class Roadmap;
class GridGraph;
struct DoorInfo;

typedef map<RegionPair, tuple<ConnectorId,Cell2D,Cell2D> > RegionConnectorMap;
typedef shared_ptr<DoorInfo> DoorInfoPtr;
typedef map<RegionId, DoorInfoPtr> RegionDoorMap;
typedef boost::multi_array<int, 2> ObstacleDistanceArray;
typedef shared_ptr<OccupancyGrid> GridPtr;
typedef vector<OutletInfo> OutletVector;

// Implementation details for top map
class TopologicalMap::MapImpl
{
public:

  /// Default constructor creates empty graph
  MapImpl(const OccupancyGrid& grid, double resolution, double door_open_prior_prob, double door_reversion_rate, double locked_door_cost, const tf::Transform& transform);

  /// Constructor that reads from a stream
  MapImpl(istream& stream, double door_open_prior_prob, double door_reversion_rate, double locked_door_cost, const tf::Transform& transform);

  /// \return Id of region containing \a p
  /// \throws UnknownCell2DException
  RegionId containingRegion(const Cell2D& p) const;

  /// \return Id of region containing point \a p
  /// \throws UnknownPointException
  RegionId containingRegion (const Point2D& p) const;

  /// \return Integer representing type of region
  /// \throws UnknownRegionException
  int regionType(const RegionId id) const;

  /// \return Door message corresponding to region
  /// \throws UnknownRegionException
  /// \throws NoDoorInRegionException
  Door regionDoor (RegionId id) const;

  /// \post Door info for region \a id has been updated to take \a msg into account.  If there's no door, one will be added, else the existing one will be updated.
  /// \throws UnknownRegionException
  /// \throws NotDoorwayRegionException
  void observeDoorMessage (RegionId id, const Door& msg);

  /// \post New evidence about attempted door traversal has been incorporated
  /// \pre \a stamp must be greater than the stamp of the last call to this function
  /// \param succeeded true iff the door was successfully traversed
  /// \param stamp time when the attempted traversal finished
  void observeDoorTraversal (RegionId id, bool succeeded, const Time& stamp);

  /// \return Probability that this door is open at the given time
  /// \pre \a stamp must be greater than the stamp of the last observation to this door (or 0 if there are no observations yet)
  /// \throws ObservationOutOfSequenceExceptioon
  /// \throws NoDoorInRegionException
  double doorOpenProb (RegionId id, const ros::Time& stamp);

  /// \return If probability of door being open is less than DOOR_OPEN_PROB_THRESHOLD, return false, otherwise return true
  /// \throws ObservationOutOfSequenceExceptioon
  /// \throws NoDoorInRegionException
  bool isDoorOpen (RegionId id, const Time& stamp);

  /// \return The id of the nearest outlet to this 2d-position
  /// \throws NoOutletException
  OutletId nearestOutlet (const Point2D& p) const;

  /// \return (copy of) stored information about a given outlet
  /// \throws UnknownOutletException
  OutletInfo outletInfo (OutletId id) const;

  /// \post Outlet is observed blocked
  /// \throws UnknownOutletException
  void observeOutletBlocked (OutletId id);

  /// \post New outlet added
  /// \return id of new outlet
  OutletId addOutlet (const OutletInfo& outlet);

  /// \return set of cells in region given id
  /// \throws UnknownRegionException
  RegionPtr regionCells (const RegionId id) const;

  /// \return connector near given point
  ConnectorId pointConnector (const Point2D& p) const;

  /// \return point corresponding to connector id
  Point2D connectorPosition (const ConnectorId id) const;

  /// \return Vector of id's of neighboring regions to region \a r
  RegionIdVector neighbors(const RegionId r) const;

  /// \return Set of all region ids.  
  const RegionIdSet& allRegions() const;

  bool isObstacle (const Point2D& p) const ;

  /// \return vector of adjacent connector ids to region \a id
  /// \throws UnknownRegionException
  vector<ConnectorId> adjacentConnectors (const RegionId id) const;

  /// \return vector of descriptions of connector cells adjacent to region \a id
  /// \throws UnknownRegionException
  vector<tuple<ConnectorId, Cell2D, Cell2D> > adjacentConnectorCells (const RegionId id) const;

  /// \return pair of ids of regions touching the given connector
  /// \throws UnknownConnectorException
  RegionPair adjacentRegions (const ConnectorId id) const;

  /// \post Set the goal point (for future distance queries) to be \a p
  void setGoal (const Point2D& p);

  /// \post Set the goal point (for future distance queries) to be center of \a c
  void setGoal (const Cell2D& p);

  /// \post Unsets the last set goal point
  void unsetGoal ();

  /// \return 1) true if there exists a path between connector \a id and goal 2) The distance (only valid if 1 is true)
  pair<bool, double> goalDistance (ConnectorId id) const;

  /// \return 1) true if there exists a path between these two points 2) the distance (only valid if 1 is true)
  pair<bool, double> getDistance (const Point2D& p1, const Point2D& p2);

  /// \return A vector of pairs.  There's one pair per connector in the containing region of p1, consisting of that connector's id 
  /// and the cost of the best path from p1 to p2 through that id
  vector<pair<ConnectorId, double> > connectorCosts (const Point2D& p1, const Point2D& p2);

  /// \return A vector of pairs.  There's one pair per connector in the containing region of p1, consisting of that connector's id 
  /// and the cost of the best path from p1 to p2 through that id
  /// \param time Door costs are measured at this time
  vector<pair<ConnectorId, double> > connectorCosts (const Point2D& p1, const Point2D& p2, const Time& time);

  /// \post New region has been added
  /// \return Id of new region
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  void removeRegion (const RegionId id);

  /// write map to \a stream in human-readable form
  void writeToStream (ostream& stream);

  /// \post Occupancy grid and outlet info written to \a filename
  void writeGridAndOutletData (const string& filename) const;

  /// write map in ppm format
  void writePpm (ostream& str) const;

  // Return cell corresponding to a given connector id and region
  Cell2D connectorCell (ConnectorId id, RegionId r) const;

  /// \post Sets the prior probability of doors being open.  Default is .1;
  /// \param prob must be between 0 and 1
  void setDoorOpenPriorProb (double prob) { ROS_ASSERT ((prob>=0.0)&&(prob<=1.0)); door_open_prior_prob_ = prob; }

  /// \post Sets the rate at which door states revert to their prior probability after an observation.
  /// \param rate must be > 0.  
  void setDoorReversionRate (double rate) { ROS_ASSERT (rate>0.0); door_reversion_rate_ = rate; }
private: 

  // During the lifetime of an instance of this class, a temporary node will exist in the connector graph at point p
  struct TemporaryRoadmapNode {
    TemporaryRoadmapNode (MapImpl* m, const Point2D& p);
    ~TemporaryRoadmapNode ();
    MapImpl* map;
    const ConnectorId id;
  };
  friend struct TemporaryRoadmapNode;
  typedef shared_ptr<TemporaryRoadmapNode> TempNodePtr;

  MapImpl(const MapImpl&);
  MapImpl& operator= (const MapImpl&);

  Cell2D containingCell(const Point2D& p) const;
  ConnectorId connectorBetween (const RegionId r1, const RegionId r2) const;
  tuple<ConnectorId, Cell2D, Cell2D> connectorCellsBetween (const RegionId r1, const RegionId r2) const;
  Point2D findBorderPoint(const Cell2D& cell1, const Cell2D& cell2) const;
  bool pointOnMap (const Point2D& p) const;
  void setDoorCost (RegionId id, const Time& t);
  void setDoorCosts (const Time& t);

  Point2D transformPoint(const Point2D& p) const;
  Point2D inverseTransformPoint(const Point2D& p) const;

  GridPtr grid_;
  ObstacleDistanceArray obstacle_distances_;

  shared_ptr<RegionGraph> region_graph_;
  shared_ptr<Roadmap> roadmap_;

  shared_ptr<GridGraph> grid_graph_;

  RegionConnectorMap region_connector_map_;

  double door_open_prior_prob_;
  double door_reversion_rate_;
  double locked_door_cost_;

  RegionDoorMap region_door_map_;

  shared_ptr<TemporaryRoadmapNode> goal_;
  
  OutletVector outlets_;
 
  const double resolution_;

  const tf::Transform& transform_;

};


    



  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_IMPL_H

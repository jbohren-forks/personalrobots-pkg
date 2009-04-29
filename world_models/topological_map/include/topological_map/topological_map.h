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
 */

#ifndef TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H
#define TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H

#include "region.h"
#include <map>
#include <vector>
#include <string>
#include <set>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>
#include <boost/tuple/tuple.hpp>
#include <robot_msgs/Door.h>
#include <topological_map/outlet_info.h>
#include <ros/time.h>

namespace topological_map
{

using std::vector;
using std::string;
using std::ostream;
using std::istream;
using std::set;
using std::pair;




typedef unsigned int RegionId;
typedef unsigned int ConnectorId;
typedef vector<RegionId> RegionIdVector;
typedef set<RegionId> RegionIdSet;
typedef pair<RegionId, RegionId> RegionPair;
typedef unsigned int OutletId;

typedef boost::multi_array<bool, 2> OccupancyGrid;
typedef OccupancyGrid::size_type occ_grid_size;

// Utilities for the occupancy grid
uint numRows(const OccupancyGrid& grid);
uint numCols(const OccupancyGrid& grid);


// Constants
const double DEFAULT_DOOR_OPEN_PRIOR_PROB = 0.95;
const double DEFAULT_DOOR_REVERSION_RATE = 1000;
const double DEFAULT_RESOLUTION=1.0;
const double DEFAULT_LOCKED_DOOR_COST=100;

/// \brief A facade object to the topological map and high-level world model information
///
/// The topological map assumes an underlying occupancy grid static map.  It includes
/// 1) A decomposition of the free cells into regions of various types
/// 2) A set of connectors, each of which is a 2d point (not cell) on the border between two regions
/// 3) A roadmap over the connectors
/// 4) For each region of type doorway, further information about the static/dynamic state of the door (not yet done)
class TopologicalMap
{
public:

  /// Default constructor makes an empty map (see also topologicalMapFromGrid)
  /// \param resolution m/cell in grid
  /// \param door_open_prior_prob Unobserved doors revert over time to this probability of being open
  /// \param door_reversion_rate If you don't observe the door for this many seconds, its probability will return about halfway to its prior value
  TopologicalMap(const OccupancyGrid&, double resolution=DEFAULT_RESOLUTION, double door_open_prior_prob=DEFAULT_DOOR_OPEN_PRIOR_PROB, 
                 double door_reversion_rate=DEFAULT_DOOR_REVERSION_RATE);

  /// Constructor that reads map from a stream
  /// \todo identify error conditions
  /// \param door_open_prior_prob Unobserved doors revert over time to this probability of being open
  /// \param door_reversion_rate If you don't observe the door for this many seconds, its probability will return about halfway to its prior value
  TopologicalMap(istream& stream, double door_open_prior_prob=DEFAULT_DOOR_OPEN_PRIOR_PROB, double door_reversion_rate=DEFAULT_DOOR_REVERSION_RATE);

  /// \post Topological map is written to \a stream in format that can be read back using the stream constructor.  All state is saved except for the currently set goal.
  /// \todo Identify error conditions
  void writeToStream (ostream& stream) const;

  /// \return Id of region containing a grid cell \a p
  /// \throws UnknownGridCellException
  RegionId containingRegion(const Cell2D& p) const;

  /// \return Id of region containing a 2d point
  /// \throws UnknownPointException
  /// \throws UnknownGridCellException
  RegionId containingRegion(const Point2D& p) const;

  /// \return Id of connector that equals a given 2d point (\a x, \a y)
  /// \throws PointNotConnectorException
  /// \throws UnknownGridCellException
  ConnectorId pointConnector (const Point2D& p) const;

  /// \return Position of connector \a id
  /// \throws UnknownConnectorException
  /// \throws UnknownGridCellException
  Point2D connectorPosition (ConnectorId id) const;

  /// \return vector of ids of connectors touching the given region
  /// \throws UnknownRegionException
  vector<ConnectorId> adjacentConnectors (RegionId id) const;

  /// \return vector (of length 2) of ids of regions touching the given connector
  /// \throws UnknownConnectorException
  RegionPair adjacentRegions (ConnectorId id) const;

  /// \return Type of this region
  /// \throws UnknownRegionException
  int regionType (RegionId id) const;

  /// \return Info about door in region \a id stored in a Door message object
  /// \throws UnknownRegionException
  robot_msgs::Door regionDoor (RegionId id) const;

  /// \post Door info for region \a id has been updated to take \a msg into account.  If there's no door, one will be added, else the existing one will be updated.
  /// \throws UnknownRegionException
  /// \throws NotDoorwayRegionException
  void observeDoorMessage (RegionId id, const robot_msgs::Door& msg);

  /// \post New evidence about attempted door traversal has been incorporated
  /// \pre \a stamp must be greater than the stamp of the last call to this function for this door
  /// \param succeeded true iff the door was successfully traversed
  /// \param stamp time when the attempted traversal finished
  /// \throws ObservationOutOfSequenceException
  /// \throws NoDoorInRegionException
  void observeDoorTraversal (RegionId id, bool succeeded, const ros::Time& stamp);

  /// \return Probability that this door is open at the given time
  /// \pre \a stamp must be greater than the stamp of the last observation to this door (or 0 if there are no observations yet)
  /// \throws ObservationOutOfSequenceExceptioon
  /// \throws NoDoorInRegionException
  double doorOpenProb (RegionId id, const ros::Time& stamp);

  /// \return The id of the nearest outlet to this point
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

  /// \return Is this point in an obstacle cell?
  /// \throws UnknownPointException
  /// \throws UnknownGridCellException
  bool isObstacle (const Point2D& p) const ;

  /// \return Vector of id's of neighboring regions to region \a r
  /// \throws UnknownRegionException
  RegionIdVector neighbors(RegionId r) const;

  /// \return (shared pointer to) set of actual grid cells in the region given \a id
  /// \throws UnknownRegionException
  RegionPtr regionCells (RegionId id) const;

  /// \return Set of all region ids.
  const RegionIdSet& allRegions() const;

  /// \return 1) true if there exists a path between these two points 2) the distance (only valid if 1 is true)
  pair<bool, double> distanceBetween (const Point2D& p1, const Point2D& p2);

  /// \return A vector of pairs.  There's one pair per connector in the containing region of p1, consisting of that connector's id 
  /// and the cost of the best path from p1 to p2 through that id
  vector<pair<ConnectorId, double> > connectorCosts (const Point2D& p1, const Point2D& p2);

  /// \post New region has been added.  Based on cell2d connectivity, the region is connected to existing regions, and connectors are added, as necessary.
  /// \return Id of new region, which will be 1+the highest previously seen region id (or 0)
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  /// \todo currently doesn't work properly with connectors
  void removeRegion (RegionId id);

  /// Output a ppm representation of the topological map to \a stream
  void writePpm (ostream& str) const;


private:

  // Forbid copy and assign
  TopologicalMap(const TopologicalMap&);
  TopologicalMap& operator= (const TopologicalMap&);

  // Avoid client dependency on implementation classes
  class MapImpl;
  boost::shared_ptr<MapImpl> map_impl_;

};





/************************************************************
 * Creation
 ************************************************************/

typedef boost::shared_ptr<TopologicalMap> TopologicalMapPtr;

/// \return shared_ptr to a new topological map generated using a bottleneck analysis of \a grid.  The region types of the returned map are either OPEN or DOORWAY
TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, double resolution, uint bottleneck_size, uint bottleneck_width, 
                                          uint bottleneck_skip, uint inflation_radius, const string& ppm_output_dir);

enum RegionType { OPEN, DOORWAY };



/************************************************************
 * Debug
 ************************************************************/


 /// \brief Print the topological map in human readable form
 ostream& operator<< (ostream& str, const TopologicalMap& c);



  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H

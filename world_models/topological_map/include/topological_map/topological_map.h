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
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>

namespace topological_map
{

using std::vector;
using std::string;
using std::ostream;
using std::pair;




typedef unsigned int RegionId;
typedef unsigned int ConnectorId;
typedef vector<RegionId> RegionIdVector;
typedef pair<RegionId, RegionId> RegionPair;

struct Point2D 
{
  Point2D(double x=0.0, double y=0.0) : x(x), y(y) {}
  double x,y;
};

ostream& operator<< (ostream& str, const Point2D& p);
bool operator== (const Point2D& p1, const Point2D& p2);




typedef boost::multi_array<bool, 2> OccupancyGrid;
typedef OccupancyGrid::size_type occ_grid_size;

// Utilities for the occupancy grid
uint numRows(const OccupancyGrid& grid);
uint numCols(const OccupancyGrid& grid);





/// \brief Represents a topological map of a 2-dimensional discrete grid decomposed into regions of various types, with connectors between them
class TopologicalMap
{
public:

  /// Default constructor makes an empty map (see also topologicalMapFromGrid)
  TopologicalMap(const OccupancyGrid&, double resolution=1.0);

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
  Point2D connectorPosition (const ConnectorId id) const;

  /// \return vector of ids of connectors touching the given region
  /// \throws UnknownRegionException
  vector<ConnectorId> adjacentConnectors (const RegionId id) const;

  /// \return vector (of length 2) of ids of regions touching the given connector
  /// \throws UnknownConnectorException
  RegionPair adjacentRegions (const ConnectorId id) const;

  /// \return Type of this region
  /// \throws UnknownRegionException
  int regionType (const RegionId id) const;

  /// \return Is this point in an obstacle cell?
  /// \throws UnknownPointException
  /// \throws UnknownGridCellException
  bool isObstacle (const Point2D& p) const ;

  /// \return Vector of id's of neighboring regions to region \a r
  /// \throws UnknownRegionException
  RegionIdVector neighbors(const RegionId r) const;

  /// \return (shared pointer to) set of actual grid cells in the region given \a id
  /// \throws UnknownRegionException
  RegionPtr regionCells (const RegionId id) const;

  /// \return Vector of all region ids.  This is a reference and may change.
  const RegionIdVector& allRegions() const;

  /// \post New region has been added.  Based on cell2d connectivity, the region is connected to existing regions, and connectors are added, as necessary.
  /// \return Id of new region, which will be 1+the highest previously seen region id (or 0)
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  /// \todo currently doesn't work properly with connectors
  void removeRegion (const RegionId id);

  /// \post Topological map is written to \a filename in format that can be read back using loadFromFile
  /// \throws FileOpenException
  void saveToFile (const string& filename) const;

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
TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, const double resolution, const uint bottleneck_size, const uint bottleneck_width, const uint bottleneck_skip, const uint inflation_radius, const string& ppm_output_dir);

enum RegionType { OPEN, DOORWAY };

/// \return shared_ptr to a topological map loaded from \a filename.  
/// \throws FileOpenException
/// \throws FileFormatException
TopologicalMapPtr loadFromFile (const string& filename);







 /************************************************************
  * Debug
  ************************************************************/


 /// \brief Print the topological map in human readable form
 ostream& operator<< (ostream& str, const TopologicalMap& c);



  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H

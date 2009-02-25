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

namespace topological_map
{

using std::map;

class RegionGraph;

typedef map<RegionPair, ConnectorId> RegionConnectorMap;

// Implementation details for top map
class TopologicalMap::MapImpl
{
public:

  /// Default constructor creates empty graph
  MapImpl(const OccupancyGrid& grid, double resolution=1.0);

  /// \return Id of region containing \a p
  /// \throws UnknownCell2DException
  RegionId containingRegion(const Cell2D& p) const;

  /// \return Id of region containing point \a p
  /// \throws UnknownPointException
  RegionId containingRegion (const Point2D& p) const;

  /// \return Integer representing type of region
  /// \throws UnknownRegionException
  int regionType(const RegionId id) const;

  /// \return set of cells in region given id
  /// \throws UnknownRegionException
  RegionPtr regionCells (const RegionId id) const;

  /// \return connector near given point
  ConnectorId pointConnector (const Point2D& p) const;

  /// \return point corresponding to connector id
  Point2D connectorPosition (const ConnectorId id) const;

  /// \return Vector of id's of neighboring regions to region \a r
  RegionIdVector neighbors(const RegionId r) const;

  /// \return Vector of all region ids
  const RegionIdVector& allRegions() const;

  bool isObstacle (const Point2D& p) const ;

  /// \return vector of adjacent connector ids to region \a id
  /// \throws UnknownRegionException
  vector<ConnectorId> adjacentConnectors (const RegionId id) const;

  /// \return pair of ids of regions touching the given connector
  /// \throws UnknownConnectorException
  RegionPair adjacentRegions (const ConnectorId id) const;

  /// \post New region has been added
  /// \return Id of new region
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  void removeRegion (const RegionId id);


private:

  MapImpl(const MapImpl&);
  MapImpl& operator= (const MapImpl&);

  Point2D getConnector(const ConnectorId id) const;
  Cell2D containingCell(const Point2D& p) const;
  Point2D cellCorner (const Cell2D& cell) const;
  ConnectorId cellConnector (const Cell2D& p) const;
  ConnectorId connectorBetween (const RegionId r1, const RegionId r2) const;
  Point2D findBorderPoint(const Cell2D& cell1, const Cell2D& cell2) const;
  bool pointOnMap (const Point2D& p) const;

  boost::shared_ptr<RegionGraph> region_graph_;

  vector<Point2D> connectors_;
  RegionConnectorMap region_connector_map_;
  
  const double resolution_;
  const OccupancyGrid& grid_;
};


  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_IMPL_H

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
 */

/**
 * \file
 *
 * Creating a topological map from known static map and door information
 *
 * \author Bhaskara Marthi
 *
 */


#include <topological_map/topological_map.h>
#include <utility>
#include <ros/console.h>
#include <ros/assert.h>
#include <topological_map/grid_graph.h>


namespace topological_map
{

using std::min;
using std::max;
using boost::tie;

// Projects points onto a fixed line segment
struct LineProjector
{
  LineProjector(const Point2D& p1, const Point2D& p2)
  {
    x1=p1.x;
    y1=p1.y;
    dx=p2.x-x1;
    dy=p2.y-y1;
    squared_dist = dx*dx+dy*dy;
  }

  pair<double, double> operator() (const Point2D& p)
  {
    double offset_x=p.x-x1;
    double offset_y=p.y-y1;

    double proportion = (offset_x*dx + offset_y*dy)/squared_dist;
    return std::make_pair(proportion, sqrt(offset_x*offset_x + offset_y*offset_y + squared_dist*proportion*proportion));
  }

  double x1, y1, dx, dy, squared_dist;
};


RegionPtr getDoorCells (const Point2D& p1, const Point2D& p2, double width, double length_error_threshold, double resolution)
{
  Cell2D c1 = pointToCell(p1,resolution);
  Cell2D c2 = pointToCell(p2,resolution);

  uint rmin = max(0,min(c1.r, c2.r)-1);
  uint rmax = max(c1.r, c2.r)+1;
  uint cmin = max(0,min(c1.c, c2.c)-1);
  uint cmax = max(c1.c, c2.c)+1;

  MutableRegionPtr region(new Region);
  LineProjector project(p1, p2);
  
  // Loop over cells in a bounding box of the line segment p1->p2
  for (uint r=rmin; r<=rmax; ++r) {
    for (uint c=cmin; c<=cmax; ++c) {

      // If the center of the cell is close to the line segment formed by p1 and p2, add the cell
      Point2D p=cellCenter(Cell2D(r, c), resolution);
      double d, l;
      tie(l,d) = project(p);
      if ((d<width) && (l/l > -length_error_threshold) && (l/l < 1+length_error_threshold)) {
        region->insert(Cell2D(r,c));
      }
    }
  }
  return region;
}


struct DoorInfo
{
  Point2D p1, p2;
};  



struct GetDoorCells
{
  GetDoorCells (double resolution, double width) : resolution(resolution), width(width) {}
  RegionPtr operator() (const DoorInfo& d) { return getDoorCells(d.p1, d.p2, width, LENGTH_ERROR_THRESHOLD, resolution); }
  const double resolution, width;
  static const double LENGTH_ERROR_THRESHOLD=.05;
};


struct CutoffDoors
{
  CutoffDoors (GridGraph& g, vector<RegionIsolator>& v) : graph(g), v(v) {}
  void operator() (RegionPtr r) { v.push_back(RegionIsolator(&graph, *r)); }
  GridGraph& graph;
  vector<RegionIsolator>& v;
};


struct Contains
{
  Contains (const Cell2D& cell) : cell(cell) {}
  bool operator() (RegionPtr region) 
  {
    return region->find(cell)!=region->end();
  }
  const Cell2D& cell;
};


struct AddRegions
{
  AddRegions (TopologicalMapPtr map, const vector<RegionPtr>& door_regions) : map(map), door_regions(door_regions) {}
  void operator() (RegionPtr region)
  {
    if (!region->empty()) {
      Cell2D cell = *(region->begin());
      RegionType type = find_if(door_regions.begin(), door_regions.end(), Contains(cell))==door_regions.end() ? OPEN : DOORWAY;
      map->addRegion(region, type);
    }
  }

  TopologicalMapPtr map;
  const vector<RegionPtr>& door_regions;
};


TopologicalMapPtr groundTruthTopologicalMap (const OccupancyGrid& grid, const vector<DoorInfo>& door_info, const double resolution, const double width)
{
  GridGraph g(grid);
  
  vector<RegionPtr> door_regions(door_info.size());
  transform(door_info.begin(), door_info.end(), door_regions.begin(), GetDoorCells(resolution, width));
  
  // This will cause the doors to be severed from the rest of the grid for the rest of this scope
  vector<RegionIsolator> v;
  for_each (door_regions.begin(), door_regions.end(), CutoffDoors(g,v));
  
  vector<MutableRegionPtr> comps = g.connectedComponents();

  TopologicalMapPtr map(new TopologicalMap(grid, resolution));
  for_each (comps.begin(), comps.end(), AddRegions(map, door_regions));

  return map;
}

} // namespace





namespace tmap=topological_map;

int main(int argc, char** argv)
{
  tmap::OccupancyGrid grid;
  std::vector<tmap::DoorInfo> doors;
  double resolution, width;
  tmap::TopologicalMapPtr m = groundTruthTopologicalMap(grid, doors, resolution, width);
}









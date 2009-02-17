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

#ifndef VISUAL_NAV_VISUAL_NAV_H
#define VISUAL_NAV_VISUAL_NAV_H

#include <vector>
#include <boost/shared_ptr.hpp>

namespace visual_nav
{

/// Represents relative or absolute 2d positions
struct Position2D
{
  Position2D (const double x, const double y) : x(x), y(y) {}
  double x, y;
};

typedef boost::shared_ptr<std::vector<int> > PathPtr;

/// Represents the roadmap produced by visual odometry, with the localization info incorporated
class VisualNavRoadmap
{
public:
  /// Default constructor creates a roadmap with a single node for the current position 
  VisualNavRoadmap();
  
  /// \post There is a new graph node at \a position
  /// \return The id of the new node
  int addNode (const Position2D& pos);

  /// \post There is an edge between nodes \a i and \a j with no label
  /// \throws \UnknownNodeId
  void addEdge (const int i, const int j);

  /// \post There is an edge from the start node to node i with the given relative position
  void addEdgeFromStart (const int i, const Position2D& relative_pos);

  /// \returns Sequence of integer labels of positions on path from start node to node \goal
  PathPtr pathToGoal (const int goal);

private:

  // Avoid client compilation dependency on implementation details
  class RoadmapImpl;
  boost::shared_ptr<RoadmapImpl> roadmap_impl_;

  // Forbid copy and assign
  VisualNavRoadmap (const VisualNavRoadmap&);
  VisualNavRoadmap& operator= (const VisualNavRoadmap&);
  
};


} // visual_nav

#endif


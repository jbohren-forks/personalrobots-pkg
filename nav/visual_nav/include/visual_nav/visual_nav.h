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
#include <string>
#include <boost/shared_ptr.hpp>
#include <visual_nav/transform.h>

namespace visual_nav
{

using boost::shared_ptr;
using std::vector;
using std::string;

/// Id's of nodes in the roadmap
typedef int NodeId;

typedef vector<NodeId> NodeVector;

typedef shared_ptr<vector<NodeId> > PathPtr;

/// Represents the roadmap produced by visual odometry, with the localization info incorporated
class VisualNavRoadmap
{
public:
  /// Default constructor creates a roadmap with a single node for the current position 
  VisualNavRoadmap();
  
  /// \post There is a new graph node at \a pose
  /// \return The id of the new node.  It is guaranteed the id returns 0 on the first call to addNode, and increases by 1 on each call.
  NodeId addNode (const Pose& pose);

  /// \post There is a new graph node at \a x, \a y, \a theta
  /// \return The id of the new node.  It is guaranteed the id returns 0 on the first call to addNode, and increases by 1 on each call.
  NodeId addNode (double x, double y, double theta=0.0);
  

  /// \post There is an edge between nodes \a i and \a j with no label
  /// \throws UnknownNodeId
  /// \throws SelfEdgeException
  /// \throws ExistingEdgeException
  void addEdge (NodeId i, NodeId j);

  /// \returns Sequence of NodeId's of positions on path from start node to node \goal
  /// \throws NoPathFoundException
  PathPtr pathToGoal (NodeId start, NodeId goal) const;

  /// \returns First point where path leaves a circle of radius \a r around its start
  /// For now, just look at the discrete waypoints on the path rather than interpolating between them to find the exact exit point
  Pose pathExitPoint (PathPtr p, double r) const;

  /// \return number of nodes
  uint numNodes () const;

  /// \return Pose of node \a i
  /// \throws UnknownNodeId
  Pose nodePose (NodeId i) const;

  /// \return vector of neighbors of node \a i
  /// \throws UnknownNodeId
  NodeVector neighbors (NodeId i) const;

  /// \return Vector of node ids in graph.
  NodeVector nodes () const;

private:

  // Avoid client dependency on implementation details
  class RoadmapImpl;
  shared_ptr<RoadmapImpl> roadmap_impl_;

  // Forbid copy and assign
  VisualNavRoadmap (const VisualNavRoadmap&);
  VisualNavRoadmap& operator= (const VisualNavRoadmap&);
  
};


typedef shared_ptr<VisualNavRoadmap> RoadmapPtr;

/// \return A pointer to a roadmap read from file f
RoadmapPtr readRoadmapFromFile(const string& filename);


} // visual_nav

#endif


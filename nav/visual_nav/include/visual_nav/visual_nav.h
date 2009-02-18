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
#include <visual_nav/transform.h>

namespace visual_nav
{

/// Id's of nodes in the roadmap
typedef int NodeId;

typedef boost::shared_ptr<std::vector<NodeId> > PathPtr;

/// Represents the roadmap produced by visual odometry, with the localization info incorporated
class VisualNavRoadmap
{
public:
  /// Default constructor creates a roadmap with a single node for the current position 
  VisualNavRoadmap();
  
  /// \post There is a new graph node at \a pose
  /// \return The id of the new node
  NodeId addNode (const Pose& pose);

  /// \post There is a new graph node with pose (\a x, \a y, \a theta)
  /// \return id of new node
  NodeId addNode (double x, double y, double theta=0.0);

  /// \post There is an edge between nodes \a i and \a j with no label
  /// \throws UnknownNodeId
  /// \throws SelfEdgeException
  /// \throws ExistingEdgeException
  /// \throws StartEdgeException
  void addEdge (const NodeId i, const NodeId j);

  /// \post There is an edge from the start node to node i with the given relative pose
  void addEdgeFromStart (const NodeId i, const Transform2D& relative_pose);

  /// \returns Sequence of NodeId's of positions on path from start node to node \goal
  PathPtr pathToGoal (const NodeId goal) const;

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


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

#include <visual_nav/roadmap_impl.h>

using std::vector;

namespace visual_nav
{


/************************************************************
 * RoadmapImpl ops
 ************************************************************/

int VisualNavRoadmap::RoadmapImpl::addNode (const Position2D& pos)
{
  add_vertex(NodeInfo(pos, next_node_id), graph_);
  return next_node_id++;
}

int VisualNavRoadmap::RoadmapImpl::addEdge (const int i, const int j)
{
  return 42;
}

void VisualNavRoadmap::RoadmapImpl::addEdgeFromStart (const int i, const Position2D& relative_pos)
{
}

PathPtr VisualNavRoadmap::RoadmapImpl::pathToGoal (const int goal_id)
{
  return PathPtr(new vector<int>);
}



/************************************************************
 * VisualNavRoadmap api
 * Constructor just creates the implementation object
 * Other functions forward ops
 ************************************************************/

VisualNavRoadmap::VisualNavRoadmap() : roadmap_impl_(new RoadmapImpl) {}

int VisualNavRoadmap::addNode (const Position2D& pos)
{
  return roadmap_impl_->addNode(pos);
}

void VisualNavRoadmap::addEdge (const int i, const int j)
{
  roadmap_impl_->addEdge(i, j);
}

void VisualNavRoadmap::addEdgeFromStart (const int i, const Position2D& relative_pos)
{
  roadmap_impl_->addEdgeFromStart(i, relative_pos);
}

PathPtr VisualNavRoadmap::pathToGoal (const int goal_id)
{
  return roadmap_impl_->pathToGoal(goal_id);
}


} // visual_nav

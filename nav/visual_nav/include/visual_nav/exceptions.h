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

#ifndef VISUAL_NAV_EXCEPTION_H
#define VISUAL_NAV_EXCEPTION_H

#include <boost/format.hpp>
#include <string>
#include <stdexcept>
#include "visual_nav.h"

namespace visual_nav
{

using boost::format;
using std::string;

/// \brief A base class for all topological_map exceptions_ 
class VisualNavException: public std::runtime_error
{ 
public:
  VisualNavException (const format& error_string) : std::runtime_error(error_string.str()) {};
  VisualNavException (const string& error_string) : std::runtime_error(error_string) {};
};


class UnknownNodeIdException: public VisualNavException
{ 
public:
  UnknownNodeIdException (const NodeId id) : VisualNavException (format("Unknown node id %1%") %id) {}
};

class ExistingEdgeException: public VisualNavException
{
public:
  ExistingEdgeException (const NodeId id1, const NodeId id2) : VisualNavException(format("Already exists an edge between nodes %1% and %2%") % id1 % id2) {}
};

class SelfEdgeException: public VisualNavException
{
public:
  SelfEdgeException (const NodeId id) : VisualNavException(format("Attempted to add self-edge to node %1%") % id) {}
};

class StartEdgeException: public VisualNavException
{
public:
  StartEdgeException (const NodeId id) : VisualNavException(format("Attempted to add edge from start edge to node %1% without specifying an offset") % id) {}
};

class UnreachableGoalException: public VisualNavException
{
public:
  UnreachableGoalException (const NodeId goal_id) : VisualNavException(format("Goal node %1% was unreachable") % goal_id) {}
};

class StartNodePoseException: public VisualNavException
{
public:
  StartNodePoseException () : VisualNavException(format("Attempted to access pose of start node")) {}
};

class NonstartRelPoseException: public VisualNavException
{
public:
  NonstartRelPoseException () : VisualNavException(format("Attempted to access relative pose field of edge not involving start node")) {}
};

class InvalidPathException: public VisualNavException
{
public:
  InvalidPathException (const NodeId id) : VisualNavException (format("Path begins at %1% instead of start node") % id) {}
};

class InsufficientlyLongPathException: public VisualNavException
{
public:
  InsufficientlyLongPathException () : VisualNavException (format("Insufficiently long path")) {}
};

class ReadRoadmapException: public VisualNavException
{
public:
  ReadRoadmapException (const string& filename) : VisualNavException (format("Unable to open %1%") % filename) {}
};







} // namespace visual_nav


#endif

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

#ifndef TOPOLOGICAL_MAP_EXCEPTION_H
#define TOPOLOGICAL_MAP_EXCEPTION_H

#include <boost/format.hpp>
#include <stdexcept>
#include <topological_map/topological_map.h>


namespace topological_map
{

/// \brief A base class for all topological_map exceptions_ 
class TopologicalMapException: public std::runtime_error
{ 
public:
  TopologicalMapException(const boost::format& error_string) : std::runtime_error(error_string.str()) {};
};

/// \brief Exception denoting unknown grid cell
class UnknownGridCellException: public TopologicalMapException
{
public:
  UnknownGridCellException(const Cell2D& p) : TopologicalMapException(boost::format("Unknown grid cell %1%") % p) {}
};

/// \brief Exception denoting unknown 2d point
class UnknownPointException: public TopologicalMapException
{
public:
  UnknownPointException (const double x, const double y) : TopologicalMapException(boost::format("Unknown 2d point (%1%, %2%)") % x % y) {}
};

/// \brief Exception when trying to add a region containing an existing gridcell
class OverlappingRegionException: public TopologicalMapException
{
public:
  OverlappingRegionException(const Cell2D& c, const RegionId id) : TopologicalMapException(boost::format("Grid cell %1% already exists in region %2%") % c % id) {}
};

/// \brief Exception for a region id that doesn't exist
class UnknownRegionException: public TopologicalMapException
{
public:
  UnknownRegionException(const RegionId id) : TopologicalMapException(boost::format("Unknown region id %1%") % id) {}
};



} // namespace topological_map


#endif

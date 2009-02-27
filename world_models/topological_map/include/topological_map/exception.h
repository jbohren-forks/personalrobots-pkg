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

using boost::format;

/// \brief A base class for all topological_map exceptions_ 
class TopologicalMapException: public std::runtime_error
{ 
public:
  TopologicalMapException(const format& error_string) : std::runtime_error(error_string.str()) {};
};

/// \brief Exception denoting unknown grid cell
class UnknownGridCellException: public TopologicalMapException
{
public:
  UnknownGridCellException(const Cell2D& p) : TopologicalMapException(format("Unknown grid cell %1%") % p) {}
};

/// \brief Exception denoting unknown 2d point
class UnknownPointException: public TopologicalMapException
{
public:
  UnknownPointException (const double x, const double y) : TopologicalMapException(format("Illegal 2d point (%1%, %2%)") % x % y) {}
  UnknownPointException (const double x, const double y, const double xmax, const double ymax) : 
    TopologicalMapException(format("Illegal 2d point (%1%, %2%) (should be nonnegative and < (%3%, %4%))") % x % y % xmax % ymax) {}
};

/// \brief Exception when trying to add a region containing an existing gridcell
class OverlappingRegionException: public TopologicalMapException
{
public:
  OverlappingRegionException(const Cell2D& c, const RegionId id) : TopologicalMapException(format("Grid cell %1% already exists in region %2%") % c % id) {}
};

/// \brief Exception for a region id that doesn't exist
class UnknownRegionException: public TopologicalMapException
{
public:
  UnknownRegionException(const RegionId id) : TopologicalMapException(format("Unknown region id %1%") % id) {}
};

/// \brief Exception for unknown connector
class UnknownConnectorException: public TopologicalMapException
{
public:
  UnknownConnectorException (double x, double y) : TopologicalMapException(format("Unknown connector for point %1%, %2%") % x % y) {}
  UnknownConnectorException (int r, int c) : TopologicalMapException(format("Unknown connector for cell %1%, %2%") % r % c) {}
};

/// \brief Exception for unknown connector id
class UnknownConnectorIdException: public TopologicalMapException
{
public:
  UnknownConnectorIdException (ConnectorId id) : TopologicalMapException(format("Unknown connector id %1%") % id) {}
};


/// \brief Exception for not being able to open top map file
class FileOpenException: public TopologicalMapException
{
public: FileOpenException (const string& filename) : TopologicalMapException(format("Unable to open file %1%") % filename) {}
};

/// \brief Exception for not being able to parse top map file
class FileFormatException: public TopologicalMapException
{
public: FileFormatException (uint line_num) : TopologicalMapException(format("Ran into trouble parsing line %1% of topological map file") % line_num) {}
};

/// \brief Exception for not finding a path in the roadmap
class NoPathFoundException: public TopologicalMapException
{
public: NoPathFoundException (ConnectorId i, ConnectorId j) : TopologicalMapException(format("No path found in roadmap between nodes %1% and %2%") % i % j) {}
};



} // namespace topological_map


#endif

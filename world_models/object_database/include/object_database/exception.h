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

#ifndef OBJECT_DATABASE_EXCEPTION_H
#define OBJECT_DATABASE_EXCEPTION_H

#include <string>
#include <boost/format.hpp>
#include <stdexcept>

using std::string;
using boost::format;

namespace object_database
{

/// \brief A base class for all object_database exceptions 
/// This inherits from ros::exception 
/// which inherits from std::runtime_exception
class ObjectDatabaseException: public std::runtime_error
{ 
public:
ObjectDatabaseException(const format& error_string) : std::runtime_error(error_string.str()) {};
};

/// \brief Exception denoting a nonexistent object type
class InvalidObjectType: public ObjectDatabaseException
{
public:
  InvalidObjectType(const string& name) : ObjectDatabaseException(format("Unknown object type %1%") %name) {}
};

/// \brief Exception denoting a nonexistent object id
class UnknownObjectId: public ObjectDatabaseException
{
public:
  UnknownObjectId(const string& type, const int ind) : ObjectDatabaseException(format("Unknown id %1% for object type %2%") %ind % type) {}
};

/// \brief Exception thrown when trying to add an object id that already exists
class DuplicateObjectId: public ObjectDatabaseException
{
public:
  DuplicateObjectId(const string& type, const int id) : ObjectDatabaseException(format("Id %1% alrady exists for type %2%") %id % type) {}
};

/// \brief Exception thrown when trying to access a nonexistent key-value pair
class UnknownKey: public ObjectDatabaseException
{
public:
  UnknownKey(const string& type, const string& key) : ObjectDatabaseException(format("Unknown key %1% for object type %2%") %key %type) {}
};


} // namespace object_database


#endif

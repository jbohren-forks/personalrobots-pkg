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

#ifndef OBJECT_DATABASE_OBJECT_DATABASE_H
#define OBJECT_DATABASE_OBJECT_DATABASE_H

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <object_database/convex_polygon.h>

using std::string;
using std::vector;

namespace object_database
{

/// Defines an object type in the object database, which consists of the type name, and a vector of keys
struct ObjectTypeDescription
{
  ObjectTypeDescription(string obj_type, vector<string> obj_keys) : object_type(obj_type), keys(obj_keys) {}

  string object_type;
  vector<string> keys;
};

/// A smart pointer to a vector of ints (that represent object ids)
typedef boost::shared_ptr<vector<int> > ObjectsPtr;


class ObjectDatabase 
{
public:

  /// Create object database with given object types
  ObjectDatabase(vector<ObjectTypeDescription> object_type_descriptions);

  // destructor not needed
  
  /// \post A new object of the given type has been added to the db
  /// \return Return the id of the new object.
  /// \throws InvalidObjectType
  int addObject (const string& object_type);
  
  /// \pre There exists no object in the db of the given type and id
  /// \post There exists one object in the db of the given type and id
  /// \throws InvalidObjectType
  /// \throws DuplicateObjectId
  void addObject (const string& object_type, const int id);

  /// \post There exists no object with type \a object_type and id \a id
  /// \throws InvalidObjectType
  /// \throws DuplicateObjectId
  void removeObject (const string& object_type, const int id);

  /// \post Geometry of object \a id of type \a object_type is a copy of \a geom
  /// \throws InvalidObjectType 
  /// \throws UnknownObjectId
  void setGeometry (const string& object_type, const int id, const ConvexPolygon& geom);

  /// \post Value of \a key for object \a id of type \a object_type equals \a value
  /// \throws InvalidObjectType
  /// \throws UnknownObjectId
  /// \throws NonexistentKey
  void setKeyValue (const string& object_type, const int id, const string& key, const double value);
  
  /// \return value of \a key for object \a id of type \a object_type
  /// \throws InvalidObjectType
  /// \throws UnknownObject
  /// \throws NonexistentKey
  double getKeyValue (const string& object_type, const int id, const string& key) const;
  
  /// \return objects of type \a object_type intersecting \a region
  ObjectsPtr getObjects (const string& object_type, const ConvexPolygon& region) const;

private:

  // Nested class used for storing individual object descriptions
  struct ObjectDescription;

  // Smart pointers to object description
  typedef boost::shared_ptr<ObjectDescription> ObjDescPtr;
  typedef boost::shared_ptr<const ObjectDescription> ObjDescConstPtr;

  /// For a given object type, map from integer id to the object description
  typedef std::map<int,ObjDescPtr> ObjectMap;

  struct AddIntersectingObjects;

  // Forbid copy and assign
  ObjectDatabase(const ObjectDatabase&);
  ObjectDatabase& operator=(const ObjectDatabase&);

  /// \return smart pointer to const description of object of the given type and id
  /// \throws InvalidObjectType
  /// \throws UnknownObject
  ObjDescConstPtr getObjectDescription (const string& object_type, const int id) const;

  /// \return smart pointer to description of object of the given type and id
  /// \throws InvalidObjectType
  /// \throws UnknownObject
  ObjDescPtr getObjectDescription (const string& object_type, const int id);

  /// \return the number of this type
  /// \throws InvalidObjectType
  int lookupType (const string& name) const;

  /// \return the index for a key of type
  /// \throws NonexistentKey
  /// \throws InvalidObjectType
  int lookupKey (const string& object_type, const string& key) const;

  /// Add object given type num and id
  /// \throws DuplicateObjectId
  void addObject (const int type_num, const int object_id);

  /// \return an unused id for this type num
  int getFreeId (const int type_num) const;

  /// \return type name for this number
  const string& typeName (const int type_num) const { return object_type_descriptions_[type_num].object_type; }

  /// Object type descriptions, sorted by name
  vector<ObjectTypeDescription> object_type_descriptions_;

  /// Object descriptions
  vector<ObjectMap> object_descriptions_;
};
   



} // namespace object_database





#endif //OBJECT_DATABASE_OBJECT_DATABASE_H

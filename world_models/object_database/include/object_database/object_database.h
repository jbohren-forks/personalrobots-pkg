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

namespace object_database
{

/// Defines an object type in the object database, which consists of the type name, and a vector of keys
struct ObjectTypeDescription
{
  ObjectTypeDescription(String object_type, String[] keys) : object_type_(object_type), keys_(keys) {}
  
  String object_type_;
  vector<String> keys_;
};


class ObjectDatabase 
{
public:

  /// Create object database with given object types
  ObjectDatabase(vector<ObjectTypeDescription> object_type_descriptions);
  ~ObjectDatabase();
  
  /// Assert the existence of a new object of given type.  
  /// \return Return the id of the new object.
  int addObject (const String& object_type);
  
  /// Assert the existence of a new object of given type, with this id.
  /// If id's already in use for this type, throws a DuplicateObjectId exception
  void addObject (const String& object_type, const int& id);

  /// Set the geometry of this object
  /// Throws an InvalidObjectType or UnknownObjectId exception if object doesn't exist.
  void setGeometry (const String& object_type, const int& id, const ConvexPolygon& geom);

  /// Set value of an existing key for this object
  /// Can throw InvalidObjectType, UnknownObjectId, or NonexistentKey exceptions.
  void setKeyValue (const String& object_type, const int& id, const String& key, const int& value);
  
  /// Get value for this key
  /// Can throw InvalidObjectType, UnknownObject or NonexistentKey exceptions.
  int getKeyValue (const String& object_type, const int& id, const String& key) const;
  
  /// Get objects of a given type
  vector<Objects> get_objects (const String& object_type) const;

  /// Get objects of a given type intersecting a given region
  vector<Objects> get_objects (const String& object_type, const ConvexPolygon& region) const;

private:

  // Forbid copy and assign
  ObjectDatabase(const ObjectDatabase&);
  ObjectDatabase& operator=(const ObjectDatabase&);

  // Todo
  
};
   



} // namespace object_database





#endif //OBJECT_DATABASE_OBJECT_DATABASE_H

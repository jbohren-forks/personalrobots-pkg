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

#include <object_database/object_database.h>
#include <algorithm>
#include <ros/console.h>
#include <object_database/exception.h>

using namespace std;

namespace object_database 
{

struct ObjectDatabase::ObjectDescription
{
  ConvexPolygon geometry;
  vector<double> key_values;
};


// Used for sorting object type descriptions by name
int operator< (const ObjectTypeDescription& d1, const ObjectTypeDescription& d2)
{
  return d1.object_type < d2.object_type;
}

int operator< (const ObjectTypeDescription& d, const string& s)
{
  return d.object_type < s;
}

int operator< (const string& s, const ObjectTypeDescription& d)
{
  return s < d.object_type;
}


// For sorting the key names within a type description
void sortKeyNames (ObjectTypeDescription& desc)
{
  sort (desc.keys.begin(), desc.keys.end());
}
  
ObjectDatabase::ObjectDatabase(vector<ObjectTypeDescription> object_type_descriptions) : object_type_descriptions_(object_type_descriptions)
{
  // Sort the type descriptions by name, and the keys within each type description
  sort (object_type_descriptions_.begin(), object_type_descriptions_.end());
  for_each (object_type_descriptions_.begin(), object_type_descriptions_.end(), sortKeyNames);
  object_descriptions_.resize(object_type_descriptions.size());
}

// Generate a new id for this object and add it
int ObjectDatabase::addObject (const string& object_type)
{
  int type_id = lookupType(object_type);
  int object_id = getFreeId(type_id);
  addObject(type_id, object_id);
  return object_id;
}

// Remove object
void ObjectDatabase::removeObject (const string& object_type, const int id)
{
  ObjectMap& object_map = object_descriptions_[lookupType(object_type)];
  ObjectMap::iterator iter = object_map.find(id);
  if (iter==object_map.end()) {
    throw UnknownObjectId(object_type, id);
  }
  else {
    object_map.erase(iter);
  }
}

void ObjectDatabase::addObject (const string& object_type, const int id)
{
  addObject(lookupType(object_type), id);
}

void ObjectDatabase::setGeometry (const string& object_type, const int id, const ConvexPolygon& geometry)
{
  getObjectDescription(object_type, id)->geometry = geometry;
}

void ObjectDatabase::setKeyValue (const string& object_type, const int id, const string& key, const double value)
{
  (getObjectDescription(object_type, id)->key_values)[lookupKey(object_type, key)]=value;
}

double ObjectDatabase::getKeyValue (const string& object_type, const int id, const string& key) const
{
  return getObjectDescription(object_type, id)->key_values[lookupKey(object_type, key)];
}


struct ObjectDatabase::AddIntersectingObjects
{
  AddIntersectingObjects (ObjectsPtr o, const ConvexPolygon& p) : objects(o), poly(p) {}
  void operator() (ObjectMap::value_type item) { if (intersects(item.second->geometry, poly)) objects->push_back(item.first); } 
  ObjectsPtr objects;
  const ConvexPolygon& poly;
};

ObjectsPtr ObjectDatabase::getObjects (const string& object_type, const ConvexPolygon& poly) const
{
  const int type_num = lookupType(object_type);
  ObjectsPtr objects(new vector<int>);
  for_each (object_descriptions_[type_num].begin(), object_descriptions_[type_num].end(), AddIntersectingObjects(objects, poly));
  return objects;
}


ObjectDatabase::ObjDescConstPtr ObjectDatabase::getObjectDescription (const string& object_type, const int id) const
{
  const ObjectMap::const_iterator desc=object_descriptions_[lookupType(object_type)].find(id);
  if (desc==object_descriptions_[lookupType(object_type)].end()) {
    throw UnknownObjectId(object_type, id);
  }
  else {
    return desc->second;
  }
}

ObjectDatabase::ObjDescPtr ObjectDatabase::getObjectDescription (const string& object_type, const int id) 
{
  const ObjectMap::iterator desc=object_descriptions_[lookupType(object_type)].find(id);
  if (desc==object_descriptions_[lookupType(object_type)].end()) {
    throw UnknownObjectId(object_type, id);
  }
  else {
    return desc->second;
  }
}

int ObjectDatabase::lookupKey (const string& object_type, const string& key) const
{
  const int type_num=lookupType(object_type);
  const ObjectTypeDescription& desc=object_type_descriptions_[type_num];
  const vector<string>::const_iterator iter=lower_bound(desc.keys.begin(), desc.keys.end(), key);
  if (iter!=desc.keys.end() && *iter==key) {
    return distance(desc.keys.begin(), iter);
  }
  else {
    throw UnknownKey(object_type, key);
  }
}

int ObjectDatabase::lookupType (const string& object_type) const
{
  const vector<ObjectTypeDescription>::const_iterator iter=lower_bound(object_type_descriptions_.begin(), object_type_descriptions_.end(), object_type);
  if (iter!=object_type_descriptions_.end() && iter->object_type==object_type) { 
    return distance(object_type_descriptions_.begin(), iter);
  }
  else {
    throw InvalidObjectType(object_type);
  }
}


void ObjectDatabase::addObject (const int type_num, const int id)
{
  if (object_descriptions_[type_num].find(id) == object_descriptions_[type_num].end()) {
    ObjDescPtr o(new ObjectDescription);
    o->key_values.resize(object_type_descriptions_[type_num].keys.size());
    object_descriptions_[type_num].insert(ObjectMap::value_type(id,o));
  }
  else {
    throw DuplicateObjectId(typeName(type_num), id);
  }
}

// Return either 1+the last id or 1 if there are none right now
int ObjectDatabase::getFreeId (const int type_num) const
{
  if (object_descriptions_[type_num].size() > 0) 
  {
    return 1+object_descriptions_[type_num].rbegin()->first;
  }
  else { 
    return 1;
  }
}



} // object_database

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
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace object_database
{

class RosObjectDatabase
{
public:
  
  RosObjectDatabase ();

  void doorMsgCallback ();
  void personMsgCallback ();

private:

  // Disallow copy and assign
  RosObjectDatabase (const RosObjectDatabase&);
  RosObjectDatabase& operator= (const RosObjectDatabase&);

  ObjectDatabase db;

  DoorMessage door_msg_;
  PersonMessage person_msg_;
};

/************************************************************
 * Constructor
 * Has db structure hardcoded in
 ************************************************************/

vector<ObjectTypeDescription> standardTypeDescriptions()
{
  typedef vector<string> Keys;

  string[] door_keys = {"blocked", "angle", "x1", "y1", "x2", "y2", "direction"};
  string[] person_keys = {"x", "y", "theta", "dx", "dy"};

  ObjectTypeDescription door_desc("door", Keys(door_keys, door_keys+7));
  ObjectTypeDescription person_desc("person", Keys(person_keys, person_keys+5));

  vector<ObjectTypeDescription> descriptions;
  descriptions.push_back(door_desc);
  descriptions.push_back(person_desc);

  return descriptions;
}

RosObjectDatabase::RosObjectDatabase : db_(standardTypeDescriptions()) {}


/************************************************************
 * Callbacks
 ************************************************************/

void RosObjectDatabase::doorMsgCallback ()
{
}

void RosObjectDatabase::personMsgCallback ()
{
}

void RosObjectDatabase::spatialQueryCallback ()
{
  // Return list of object ids of a given type in a given region
}

void RosObjectDatabase::objectKeyValueCallback ()
{
  // Return value of a key of an object
}

} // namespace object_database










int main (int argc, char** argv)
{
  return 0;
}

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
 *
 */

#include <topological_map/topological_map.h>
#include <topological_map/door_info.h>

namespace topological_map
{

using std::endl;

DoorInfo::DoorInfo (istream& str)
{
  Door msg;
  str >> msg.frame_p1.x >> msg.frame_p1.y >> msg.frame_p1.z;
  str >> msg.frame_p2.x >> msg.frame_p2.y >> msg.frame_p2.z;
  str >> msg.door_p1.x >> msg.door_p1.y >> msg.door_p1.z;
  str >> msg.door_p2.x >> msg.door_p2.y >> msg.door_p2.z;
  str >> msg.handle.x >> msg.handle.y >> msg.handle.z;
  str >> msg.height;
  str >> msg.hinge;
  str >> msg.rot_dir;
  // For now ignore weight, door_boundary, normal

  msg_ = msg;
}

void DoorInfo::writeToStream (ostream& str) const
{
  str << endl << msg_.frame_p1.x << " " << msg_.frame_p1.y << " " << msg_.frame_p1.z;
  str << endl << msg_.frame_p2.x << " " << msg_.frame_p2.y << " " << msg_.frame_p2.z;
  str << endl << msg_.door_p1.x << " " << msg_.door_p1.y << " " << msg_.door_p1.z;
  str << endl << msg_.door_p2.x << " " << msg_.door_p2.y << " " << msg_.door_p2.z;
  str << endl << msg_.handle.x << " " << msg_.handle.y << " " << msg_.handle.z;
  str << endl << msg_.height << " " << msg_.hinge << " " << msg_.rot_dir;
}





Door DoorInfo::getDoorMessage () const
{
  return msg_;
}

void DoorInfo::observeDoorMessage (const Door& msg) 
{
  msg_=msg;
}


}

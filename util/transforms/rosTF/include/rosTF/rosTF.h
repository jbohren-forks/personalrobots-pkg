//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

/** \mainpage This is a ROS wrapper for the libTF library. 
 * It provides a server and client class which can be created
 * to make publishing and recieving transform information very 
 * simple.  
 * There are test server and test client code in the aptly named
 * subdirectories.  
 */

#include <iostream>
#include "ros/node.h"
#include "std_msgs/MsgTransformEuler.h"
#include "std_msgs/MsgTransformDH.h"
#include "std_msgs/MsgTransformQuaternion.h"
#include "libTF/libTF.h"

/** \brief A basic ROS client library for libTF
 * This inherits from libTF and will automatically
 * push incoming date into the library.  
 * The accessors remain available as before.  
 */
class rosTFClient : public libTF::TransformReference
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode);

  //Call back functions
  void receiveEuler();
  void receiveDH();
  void receiveQuaternion();

 private:
  // A reference to the active ros::node to allow setting callbacks
  ros::node & myNode; 
  //Temporary storage for callbacks(todo check threadsafe? make scoped in call?)
  MsgTransformEuler eulerIn;
  MsgTransformDH dhIn;
  MsgTransformQuaternion quaternionIn;

};

/** \brief A simple class to broadcast transforms
 * This class properly broadcasts transforms without
 * requiring the user to deal with any communication
 * beyond a function call.
 */
class rosTFServer
{
 public:
  //Constructor
  rosTFServer(ros::node & rosnode);
  /** \brief Send a Transform with Euler Angles */
  void sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs);
  /** \brief Send a transform using DH Parameters */
  void sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs);
  /** \brief Send a transform using Quaternion notation */
  void sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs);

 private:
  //ros::node reference to allow setting callbacks
  ros::node & myNode;
  //Temp variables for in transit. (todo check threadsafe? make scoped in call?)
  MsgTransformEuler eulerOut;
  MsgTransformDH dhOut;
  MsgTransformQuaternion quaternionOut;

};

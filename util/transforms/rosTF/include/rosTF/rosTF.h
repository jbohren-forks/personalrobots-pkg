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
#include "std_msgs/TransformEuler.h"
#include "std_msgs/TransformDH.h"
#include "std_msgs/TransformQuaternion.h"
#include "libTF/libTF.h"
#include "std_msgs/PointCloudFloat32.h"
#include "namelookup/NameToNumber.h"

//TODO FIXME REMOVE WHEN FRAME ID SERVER IS IMPLEMENTED
#define FRAMEID_MAP 1
#define FRAMEID_ROBOT 2
#define FRAMEID_ODOM 3
#define FRAMEID_LASER 4
#define FRAMEID_TILT_BASE 5
#define FRAMEID_TILT_STAGE 6
#define FRAMEID_LASER2 7

/** \brief A basic ROS client library for libTF
 * This inherits from libTF and will automatically
 * push incoming date into the library.  
 * The accessors remain available as before.  
 */
class rosTFClient : public libTF::TransformReference
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode, bool interpolating = true, unsigned long long max_cache_time = libTF::TransformReference::DEFAULT_CACHE_TIME, unsigned long long max_extrapolation_distance = libTF::TransformReference::DEFAULT_MAX_EXTRAPOLATION_DISTANCE);

  //  PointCloudFloat32 transformPointCloud(unsigned int target_frame, const PointCloudFloat32 & cloudIn); // todo switch after ticket:232
  std_msgs::PointCloudFloat32 transformPointCloud(unsigned int target_frame, std_msgs::PointCloudFloat32 & cloudIn);

  //Call back functions
  void receiveEuler();
  void receiveDH();
  void receiveQuaternion();

 private:
  // A reference to the active ros::node to allow setting callbacks
  ros::node & myNode; 
  //Temporary storage for callbacks(todo check threadsafe? make scoped in call?)
  std_msgs::TransformEuler eulerIn;
  std_msgs::TransformDH dhIn;
  std_msgs::TransformQuaternion quaternionIn;

  pthread_mutex_t cb_mutex;

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
  void sendInverseEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs);
  void sendPose(libTF::TFPose pose, unsigned int parent);
  void sendInversePose(libTF::TFPose pose, unsigned int parent);
  /** \brief Send a transform using DH Parameters */
  void sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs);
  /** \brief Send a transform using Quaternion notation */
  void sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs);


  void nameLookup(std::string astring);
 private:
  //ros::node reference to allow setting callbacks
  ros::node & myNode;
  
  bool checkInvalidFrame(unsigned int);
  
};

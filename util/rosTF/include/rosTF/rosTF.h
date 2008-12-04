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

#ifndef ROSTF_HH
#define ROSTF_HH
#include <iostream>
#include "ros/node.h"
#include "rosTF/TransformArray.h"
#include "libTF/libTF.h"
#include "std_msgs/PointCloud.h"
#include "laser_scan/laser_scan.h"


/** \brief A basic ROS client library for libTF
 * This inherits from libTF and will automatically
 * push incoming date into the library.  
 * The accessors remain available as before.  
 */
class rosTFClient : public libTF::TransformReference
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode, bool interpolating = true, uint64_t max_cache_time = libTF::TransformReference::DEFAULT_CACHE_TIME, uint64_t max_extrapolation_distance = libTF::TransformReference::DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  //Destructor
  ~rosTFClient();

  //  PointCloud transformPointCloud(unsigned int target_frame, const PointCloud & cloudIn); // todo switch after ticket:232
  std_msgs::PointCloud transformPointCloud(const std::string& target_frame, const std_msgs::PointCloud & cloudIn);
  std_msgs::PointCloud transformPointCloud(unsigned int target_frame, const std_msgs::PointCloud & cloudIn);
  void transformPointCloud(const std::string & target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::PointCloud & cloudIn) __attribute__((deprecated));
  void transformPointCloud(unsigned int target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::PointCloud & cloudIn) __attribute__((deprecated));

  void transformLaserScanToPointCloud(const std::string& target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::LaserScan & scanIn) __attribute__((deprecated));
  void transformLaserScanToPointCloud(unsigned int target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::LaserScan & scanIn) __attribute__((deprecated));

  /** @brief Call back function for receiving on ROS */
  void receiveArray();


  /*********** Accessors *************/
  /* Unhide base class functions which I'm about to hide otherwise*/
  using TransformReference::getMatrix;

  /** \brief Get the transform between two frames by frame name
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * 
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  NEWMAT::Matrix getMatrix(const std::string & target_frame, const std::string & source_frame, ros::Time time) __attribute__((deprecated));


 private:
  // A reference to the active ros::node to allow setting callbacks
  ros::node & myNode; 
  //Temporary storage for callbacks(todo check threadsafe? make scoped in call?)
  rosTF::TransformArray tfArrayIn;

  /** @brief A helper class for projecting laser scans */
  laser_scan::LaserProjection projector_;

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
  void sendEuler(const std::string & frame, const std::string & parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime) __attribute__((deprecated));
  /** \brief Send a Transform with Euler Angles using libTF syntax */
  void sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs) __attribute__((deprecated));

  /** \brief Send a transform from parent to frame when frame to parent is known */
  void sendInverseEuler(const std::string & frame, const std::string & parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime) __attribute__((deprecated));
  /** \brief Send a transform from parent to frame when frame to parent is known using libTF syntax */
  void sendInverseEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs) __attribute__((deprecated));

  /** \brief Send a transform using TFPose syntax */
  void sendPose(libTF::TFPose pose, const std::string & parent) __attribute__((deprecated));
  /** \brief Send a transform using TFPose and libTF syntax */
  void sendPose(libTF::TFPose pose, unsigned int parent) __attribute__((deprecated));

  /** \brief Send a transform using TFPose when frame to parent is known*/
  void sendInversePose(libTF::TFPose pose, const std::string & parent) __attribute__((deprecated));
  /** \brief Send a transform using TFPose and libTF syntax when frame to parent is known*/
  void sendInversePose(libTF::TFPose pose, unsigned int parent) __attribute__((deprecated));

  /** \brief Send a transform using DH Parameters */
  void sendDH(const std::string & frame, const std::string & parent, double length, double twist, double offset, double angle, ros::Time rostime) __attribute__((deprecated));
  /** \brief Send a transform using DH Parameters and libTF syntax*/
  void sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs) __attribute__((deprecated));

  /** \brief Send a transform using Quaternion notation */
  void sendQuaternion(const std::string & frame, const std::string & parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, ros::Time rostime) __attribute__((deprecated));
  /** \brief Send a transform using Quaternion notation and libTF syntax*/
  void sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs) __attribute__((deprecated));


  /** \brief Send a Transform with 4x4 Matrix */
  void sendMatrix(const std::string & frame, const std::string & parent, NEWMAT::Matrix matrix, ros::Time rostime) __attribute__((deprecated));
  /** \brief Send a Transform with 4x4 Matrix */
  void sendMatrix(unsigned int frame, unsigned int parent, NEWMAT::Matrix matrix, ros::Time rostime) __attribute__((deprecated));

 private:
  //ros::node reference to allow setting callbacks
  ros::node & myNode;
  
  
};
#endif //ROSTF_HH

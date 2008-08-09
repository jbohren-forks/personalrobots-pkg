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
#include "std_msgs/TransformEuler.h"
#include "std_msgs/TransformDH.h"
#include "std_msgs/TransformQuaternion.h"
#include "std_msgs/TransformMatrix.h"
#include "libTF/libTF.h"
#include "std_msgs/PointCloudFloat32.h"
#include "namelookup/nameLookupClient.hh"
#include "namelookup/NameToNumber.h"
#include "laser_scan_utils/laser_scan.h"


/** \brief A basic ROS client library for libTF
 * This inherits from libTF and will automatically
 * push incoming date into the library.  
 * The accessors remain available as before.  
 */
class rosTFClient : public libTF::TransformReference, public nameLookupClient
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode, bool interpolating = true, unsigned long long max_cache_time = libTF::TransformReference::DEFAULT_CACHE_TIME, unsigned long long max_extrapolation_distance = libTF::TransformReference::DEFAULT_MAX_EXTRAPOLATION_DISTANCE);

  //  PointCloudFloat32 transformPointCloud(unsigned int target_frame, const PointCloudFloat32 & cloudIn); // todo switch after ticket:232
  std_msgs::PointCloudFloat32 transformPointCloud(std::string target_frame, const std_msgs::PointCloudFloat32 & cloudIn);
  std_msgs::PointCloudFloat32 transformPointCloud(unsigned int target_frame, const std_msgs::PointCloudFloat32 & cloudIn);
  void transformPointCloud(std::string target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::PointCloudFloat32 & cloudIn);
  void transformPointCloud(unsigned int target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::PointCloudFloat32 & cloudIn);

  void transformLaserScanToPointCloud(std::string target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::LaserScan & scanIn);
  void transformLaserScanToPointCloud(unsigned int target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::LaserScan & scanIn);

  //Call back functions
  void receiveEuler();
  void receiveDH();
  void receiveQuaternion();
  void receiveMatrix();


  /*********** Accessors *************/
  /* Unhide base class functions which I'm about to hide otherwise*/
  using TransformReference::getMatrix;
  using TransformReference::transformPoint;
  using TransformReference::transformPoint2D;
  using TransformReference::transformVector;
  using TransformReference::transformVector2D;
  using TransformReference::transformEulerYPR;
  using TransformReference::transformYaw;
  using TransformReference::transformPose;
  using TransformReference::transformPose2D;
  using TransformReference::viewChain;

  /** \brief Get the transform between two frames by frame name
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * 
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  NEWMAT::Matrix getMatrix(std::string target_frame, std::string source_frame, ros::Time time);


  
  /** \brief Transform a point to a different frame */
  libTF::TFPoint transformPoint(std::string target_frame, const libTF::TFPoint & point_in);
  /** \brief Transform a 2D point to a different frame */
  libTF::TFPoint2D transformPoint2D(std::string target_frame, const libTF::TFPoint2D & point_in);
  /** \brief Transform a vector to a different frame */
  libTF::TFVector transformVector(std::string target_frame, const libTF::TFVector & vector_in);
  /** \brief Transform a 2D vector to a different frame */
  libTF::TFVector2D transformVector2D(std::string target_frame, const libTF::TFVector2D & vector_in);
  /** \brief Transform Euler angles between frames */
  libTF::TFEulerYPR transformEulerYPR(std::string target_frame, const libTF::TFEulerYPR & euler_in);
  /** \brief Transform Yaw between frames. Useful for 2D navigation */
  libTF::TFYaw transformYaw(std::string target_frame, const libTF::TFYaw & euler_in);
  /** \brief Transform a 6DOF pose.  (x, y, z, yaw, pitch, roll). */
  libTF::TFPose transformPose(std::string target_frame, const libTF::TFPose & pose_in);
  /** \brief Transform a planar pose, x,y,yaw */
  libTF::TFPose2D transformPose2D(std::string target_frame, const libTF::TFPose2D & pose_in);

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  std::string viewChain(std::string target_frame, std::string source_frame);



 private:
  // A reference to the active ros::node to allow setting callbacks
  ros::node & myNode; 
  //Temporary storage for callbacks(todo check threadsafe? make scoped in call?)
  std_msgs::TransformEuler eulerIn;
  std_msgs::TransformDH dhIn;
  std_msgs::TransformQuaternion quaternionIn;
  std_msgs::TransformMatrix matrixIn;

  laser_scan::LaserProjection projector_;

  pthread_mutex_t cb_mutex;

};

/** \brief A simple class to broadcast transforms
 * This class properly broadcasts transforms without
 * requiring the user to deal with any communication
 * beyond a function call.
 */
class rosTFServer : public nameLookupClient
{
 public:
  //Constructor
  rosTFServer(ros::node & rosnode);
  /** \brief Send a Transform with Euler Angles */
  void sendEuler(std::string frame, std::string parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime);
  /** \brief Send a Transform with Euler Angles using libTF syntax */
  void sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs);

  /** \brief Send a transform from parent to frame when frame to parent is known */
  void sendInverseEuler(std::string frame, std::string parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime);
  /** \brief Send a transform from parent to frame when frame to parent is known using libTF syntax */
  void sendInverseEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs);

  /** \brief Send a transform using TFPose syntax */
  void sendPose(libTF::TFPose pose, std::string parent);
  /** \brief Send a transform using TFPose and libTF syntax */
  void sendPose(libTF::TFPose pose, unsigned int parent);

  /** \brief Send a transform using TFPose when frame to parent is known*/
  void sendInversePose(libTF::TFPose pose, std::string parent);
  /** \brief Send a transform using TFPose and libTF syntax when frame to parent is known*/
  void sendInversePose(libTF::TFPose pose, unsigned int parent);

  /** \brief Send a transform using DH Parameters */
  void sendDH(std::string frame, std::string parent, double length, double twist, double offset, double angle, ros::Time rostime);
  /** \brief Send a transform using DH Parameters and libTF syntax*/
  void sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs);

  /** \brief Send a transform using Quaternion notation */
  void sendQuaternion(std::string frame, std::string parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, ros::Time rostime);
  /** \brief Send a transform using Quaternion notation and libTF syntax*/
  void sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs);


  /** \brief Send a Transform with 4x4 Matrix */
  void sendMatrix(std::string frame, std::string parent, NEWMAT::Matrix matrix, ros::Time rostime);
  /** \brief Send a Transform with 4x4 Matrix */
  void sendMatrix(unsigned int frame, unsigned int parent, NEWMAT::Matrix matrix, ros::Time rostime);


 private:
  //ros::node reference to allow setting callbacks
  ros::node & myNode;
  
  bool checkInvalidFrame(unsigned int);
  
};
#endif //ROSTF_HH

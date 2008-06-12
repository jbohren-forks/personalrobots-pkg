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

#include "rosTF/rosTF.h"

rosTFClient::rosTFClient(ros::node & rosnode, 
                         bool interpolating,
			 unsigned long long max_cache_time,
			 unsigned long long max_extrapolation_distance):
  TransformReference(interpolating,
                     max_cache_time,
                     max_extrapolation_distance),
  myNode(rosnode)
{
  myNode.subscribe("TransformEuler", eulerIn, &rosTFClient::receiveEuler, this,100);
  myNode.subscribe("TransformDH", dhIn, &rosTFClient::receiveDH, this);
  myNode.subscribe("TransformQuaternion", quaternionIn, &rosTFClient::receiveQuaternion, this);

};

//PointCloudFloat32 rosTFClient::transformPointCloud(unsigned int target_frame, const std_msgs::PointCloudFloat32 & cloudIn) //todo add back const when get_pts_size() is const ticket:232
std_msgs::PointCloudFloat32 rosTFClient::transformPointCloud(unsigned int target_frame,  std_msgs::PointCloudFloat32 & cloudIn)
{
  NEWMAT::Matrix transform = getMatrix(target_frame, cloudIn.header.frame_id, cloudIn.header.stamp.sec * 1000000000ULL + cloudIn.header.stamp.nsec);

  NEWMAT::Matrix matIn(4,cloudIn.get_pts_size());


  //TODO optimize with pointer accessors
   for (unsigned int i = 1; i <= cloudIn.get_pts_size();i++) 
    { 
      matIn(1,i) = cloudIn.pts[i].x;
      matIn(2,i) = cloudIn.pts[i].y;
      matIn(3,i) = cloudIn.pts[i].z;
      matIn(4,i) = 1;
    };

  NEWMAT::Matrix matOut = transform * matIn;

  std_msgs::PointCloudFloat32 cloudOut = cloudIn; //Get everything from cloudIn

  //Override the positions
  cloudOut.header.frame_id = target_frame;
   for (unsigned int i = 1; i <= cloudIn.get_pts_size();i++) 
    { 
      cloudOut.pts[i-1].x = matOut(1,i);
      cloudOut.pts[i-1].y = matOut(2,i);
      cloudOut.pts[i-1].z = matOut(3,i);
    };

  return cloudOut;
};



void rosTFClient::receiveEuler()
{
  setWithEulers(eulerIn.frame, eulerIn.parent, eulerIn.x, eulerIn.y, eulerIn.z, eulerIn.yaw, eulerIn.pitch, eulerIn.roll, eulerIn.header.stamp.sec * 1000000000ULL + eulerIn.header.stamp.nsec);
  //std::cout << "received euler frame: " << eulerIn.frame << " with parent:" << eulerIn.parent << "time " << eulerIn.header.stamp.sec * 1000000000ULL + eulerIn.header.stamp.nsec << std::endl;
};

void rosTFClient::receiveDH()
{
  setWithDH(dhIn.frame, dhIn.parent, dhIn.length, dhIn.twist, dhIn.offset, dhIn.angle, dhIn.header.stamp.sec * 1000000000ULL + dhIn.header.stamp.nsec);
  std::cout << "recieved DH frame: " << dhIn.frame << " with parent:" << dhIn.parent << std::endl;
};

void rosTFClient::receiveQuaternion()
{
  setWithQuaternion(quaternionIn.frame, quaternionIn.parent, quaternionIn.xt, quaternionIn.yt, quaternionIn.zt, quaternionIn.xr, quaternionIn.yr, quaternionIn.zr, quaternionIn.w, quaternionIn.header.stamp.sec * 1000000000ULL + quaternionIn.header.stamp.nsec);
  std::cout << "recieved quaternion frame: " << quaternionIn.frame << " with parent:" << quaternionIn.parent << std::endl;
};


rosTFServer::rosTFServer(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.advertise<std_msgs::TransformEuler>("TransformEuler");
  myNode.advertise<std_msgs::TransformDH>("TransformDH");
  myNode.advertise<std_msgs::TransformQuaternion>("TransformQuaternion");

};


void rosTFServer::sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs)
{
  std_msgs::TransformEuler eulerOut;
  eulerOut.frame = frame;
  eulerOut.parent = parent;
  eulerOut.x = x;
  eulerOut.y = y;
  eulerOut.z = z;
  eulerOut.yaw = yaw;
  eulerOut.pitch = pitch;
  eulerOut.roll = roll;
  eulerOut.header.stamp.sec = secs;
  eulerOut.header.stamp.nsec = nsecs;

  myNode.publish("TransformEuler", eulerOut);

};

void rosTFServer::sendInverseEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs)
{ 
  std_msgs::TransformEuler eulerOut;
  //Invert the transform
  libTF::Euler3D odomeuler = libTF::Pose3D::eulerFromMatrix(libTF::Pose3D::matrixFromEuler(x, y, z, yaw, pitch, roll).i());
  
  eulerOut.frame = frame;
  eulerOut.parent = parent;
  eulerOut.x = odomeuler.x;
  eulerOut.y = odomeuler.y;
  eulerOut.z = odomeuler.z;
  eulerOut.yaw = odomeuler.yaw;
  eulerOut.pitch = odomeuler.pitch;
  eulerOut.roll = odomeuler.roll;
  eulerOut.header.stamp.sec = secs;
  eulerOut.header.stamp.nsec = nsecs;

  myNode.publish("TransformEuler", eulerOut);
 
};

void rosTFServer::sendPose(libTF::TFPose pose, unsigned int parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendEuler(pose.frame, parent, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, secs, nsecs);  
};


void rosTFServer::sendInversePose(libTF::TFPose pose, unsigned int parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendInverseEuler(pose.frame, parent, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, secs, nsecs);  
};


void rosTFServer::sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs)
{
  std_msgs::TransformDH dhOut;

  dhOut.frame = frame;
  dhOut.parent = parent;
  dhOut.length = length;
  dhOut.twist = twist;
  dhOut.offset = offset;
  dhOut.angle = angle;
  dhOut.header.stamp.sec = secs;
  dhOut.header.stamp.nsec = nsecs;

  myNode.publish("TransformDH", dhOut);

};

void rosTFServer::sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs)
{
  std_msgs::TransformQuaternion quaternionOut;
  quaternionOut.frame = frame;
  quaternionOut.parent = parent;
  quaternionOut.xt = xt;
  quaternionOut.yt = yt;
  quaternionOut.zt = zt;
  quaternionOut.xr = xr;
  quaternionOut.yr = yr;
  quaternionOut.zr = zr;
  quaternionOut.w = w;
  quaternionOut.header.stamp.sec = secs;
  quaternionOut.header.stamp.nsec = nsecs;

  myNode.publish("TransformQuaternion", quaternionOut);

};

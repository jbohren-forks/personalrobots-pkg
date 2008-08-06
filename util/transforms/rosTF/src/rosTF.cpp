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
  nameLookupClient(rosnode),
  myNode(rosnode)
{
  //  printf("Constructed rosTF\n");
  myNode.subscribe("TransformEuler", eulerIn, &rosTFClient::receiveEuler, this,100);
  myNode.subscribe("TransformDH", dhIn, &rosTFClient::receiveDH, this);
  myNode.subscribe("TransformQuaternion", quaternionIn, &rosTFClient::receiveQuaternion, this);

};


void rosTFClient::transformPointCloud(std::string target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::PointCloudFloat32 & cloudIn)
{
    transformPointCloud(lookup(target_frame), cloudOut, cloudIn);
}

void rosTFClient::transformPointCloud(unsigned int target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::PointCloudFloat32 & cloudIn)
{
  NEWMAT::Matrix transform = TransformReference::getMatrix(target_frame, cloudIn.header.frame_id, cloudIn.header.stamp.sec * 1000000000ULL + cloudIn.header.stamp.nsec);

  unsigned int length = cloudIn.get_pts_size();

  NEWMAT::Matrix matIn(4, length);
  
  double * matrixPtr = matIn.Store();
  
  for (unsigned int i = 0; i < length ; i++) 
    { 
      matrixPtr[i] = cloudIn.pts[i].x;
      matrixPtr[length +i] = cloudIn.pts[i].y;
      matrixPtr[2 * length + i] = cloudIn.pts[i].z;
      matrixPtr[3 * length + i] = 1;
    };
  
  NEWMAT::Matrix matOut = transform * matIn;
  
  // Copy relevant data from cloudIn, if needed
  if (&cloudIn != &cloudOut)
  {
      cloudOut.header = cloudIn.header;
      cloudOut.set_pts_size(length);  
      cloudOut.set_chan_size(cloudIn.get_chan_size());
      for (unsigned int i = 0 ; i < cloudIn.get_chan_size() ; ++i)
	  cloudOut.chan[i] = cloudIn.chan[i];
  }
  
  matrixPtr = matOut.Store();
  
  //Override the positions
  cloudOut.header.frame_id = target_frame;
  for (unsigned int i = 0; i < length ; i++) 
    { 
      cloudOut.pts[i].x = matrixPtr[i];
      cloudOut.pts[i].y = matrixPtr[1*length + i];
      cloudOut.pts[i].z = matrixPtr[2*length + i];
    };
}

std_msgs::PointCloudFloat32 rosTFClient::transformPointCloud(std::string target_frame,  const std_msgs::PointCloudFloat32 & cloudIn)
{
  return transformPointCloud(lookup(target_frame), cloudIn);
};


//PointCloudFloat32 rosTFClient::transformPointCloud(unsigned int target_frame, const std_msgs::PointCloudFloat32 & cloudIn) //todo add back const when get_pts_size() is const ticket:232
std_msgs::PointCloudFloat32 rosTFClient::transformPointCloud(unsigned int target_frame,  const std_msgs::PointCloudFloat32 & cloudIn)
{
    std_msgs::PointCloudFloat32 cloudOut;
    transformPointCloud(target_frame, cloudOut, cloudIn);
    return cloudOut;    
};



void rosTFClient::transformLaserScanToPointCloud(std::string target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::LaserScan & scanIn)
{
  transformLaserScanToPointCloud(lookup(target_frame), cloudOut, scanIn);
}

void rosTFClient::transformLaserScanToPointCloud(unsigned int target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::LaserScan &scanIn)
{
  cloudOut.header = scanIn.header;
  cloudOut.set_pts_size(scanIn.ranges_size);

  libTF::TFPoint pointIn;
  libTF::TFPoint pointOut;

  pointIn.frame = scanIn.header.frame_id;
  


  ///\todo this 
  std_msgs::PointCloudFloat32 intermediate; //optimize out
  projector_.projectLaser(scanIn, intermediate, true);
  unsigned int count = 0;
  for (unsigned int i = 0; i < scanIn.ranges_size; i++)
  {
    if (scanIn.ranges[i] <= scanIn.range_max 
        && scanIn.ranges[i] >= scanIn.range_min) //only when valid
    {
      pointIn.time = scanIn.header.stamp.to_ull() + (unsigned long long) (scanIn.time_increment * 1000000000);
      pointIn.x = intermediate.pts[i].x;
      pointIn.y = intermediate.pts[i].y;
      pointIn.z = intermediate.pts[i].z;
      
      pointOut = transformPoint(target_frame, pointIn);
      cloudOut.pts[count].x  = pointOut.x;
      cloudOut.pts[count].y  = pointOut.y;
      cloudOut.pts[count].z  = pointOut.z;

    }
    
  }
}



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


NEWMAT::Matrix rosTFClient::getMatrix(std::string target_frame, std::string source_frame, ros::Time time)
{ return getMatrix(lookup(target_frame), lookup(source_frame), time.to_ull());};


libTF::TFPoint rosTFClient::transformPoint(std::string target_frame, const libTF::TFPoint & point_in)
{ return transformPoint(lookup(target_frame), point_in);};

libTF::TFPoint2D rosTFClient::transformPoint2D(std::string target_frame, const libTF::TFPoint2D & point_in)
{ return transformPoint2D(lookup(target_frame), point_in);};

libTF::TFVector rosTFClient::transformVector(std::string target_frame, const libTF::TFVector & vector_in)
{ return transformVector(lookup(target_frame), vector_in);};

libTF::TFVector2D rosTFClient::transformVector2D(std::string target_frame, const libTF::TFVector2D & vector_in)
{ return transformVector2D(lookup(target_frame), vector_in);};

libTF::TFEulerYPR rosTFClient::transformEulerYPR(std::string target_frame, const libTF::TFEulerYPR & euler_in)
{ return transformEulerYPR(lookup(target_frame), euler_in);};

libTF::TFYaw rosTFClient::transformYaw(std::string target_frame, const libTF::TFYaw & euler_in)
{ return transformYaw(lookup(target_frame), euler_in);};

libTF::TFPose rosTFClient::transformPose(std::string target_frame, const libTF::TFPose & pose_in)
{ return transformPose(lookup(target_frame), pose_in);};

libTF::TFPose2D rosTFClient::transformPose2D(std::string target_frame, const libTF::TFPose2D & pose_in)
{ return transformPose2D(lookup(target_frame), pose_in);};

std::string rosTFClient::viewChain(std::string target_frame, std::string source_frame)
{ return viewChain(lookup(target_frame), lookup(source_frame));};




rosTFServer::rosTFServer(ros::node & rosnode):
  nameLookupClient(rosnode),
  myNode(rosnode)
{
  myNode.advertise<std_msgs::TransformEuler>("TransformEuler");
  myNode.advertise<std_msgs::TransformDH>("TransformDH");
  myNode.advertise<std_msgs::TransformQuaternion>("TransformQuaternion");

};

void rosTFServer::sendEuler(std::string frame, std::string parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime)
{
  sendEuler(lookup(frame), lookup(parent), x,y,z,yaw,pitch,roll, rostime.sec, rostime.nsec);
};

void rosTFServer::sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs)
{
  if (!checkInvalidFrame(frame))
    return;

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

void rosTFServer::sendInverseEuler(std::string frame, std::string parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime)
{
  sendInverseEuler(lookup(frame), lookup(parent), x,y,z,yaw,pitch,roll, rostime.sec, rostime.nsec);
};

void rosTFServer::sendInverseEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs)
{ 
  if (!checkInvalidFrame(frame))
    return;

  std_msgs::TransformEuler eulerOut;
  //Invert the transform
  libTF::Pose3D::Euler odomeuler  = libTF::Pose3D::eulerFromMatrix(libTF::Pose3D::matrixFromEuler(x, y, z, yaw, pitch, roll).i()); //todo optimize
  libTF::Pose3D::Position odompos = libTF::Pose3D::positionFromMatrix(libTF::Pose3D::matrixFromEuler(x, y, z, yaw, pitch, roll).i());
  
  eulerOut.frame = frame;
  eulerOut.parent = parent;
  eulerOut.x = odompos.x;
  eulerOut.y = odompos.y;
  eulerOut.z = odompos.z;
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

void rosTFServer::sendPose(libTF::TFPose pose, std::string parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendEuler(pose.frame, lookup(parent), pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, secs, nsecs);  
};

void rosTFServer::sendInversePose(libTF::TFPose pose, std::string parent)
{
  sendInversePose(pose, lookup(parent));
};

void rosTFServer::sendInversePose(libTF::TFPose pose, unsigned int parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendInverseEuler(pose.frame, parent, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, secs, nsecs);  
};


void rosTFServer::sendDH(std::string frame, std::string parent, double length, double twist, double offset, double angle, ros::Time rostime)
{
  sendDH(lookup(frame), lookup(parent), length, twist, offset, angle, rostime.sec, rostime.nsec);
};

void rosTFServer::sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs)
{
  if (!checkInvalidFrame(frame))
    return;

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

void rosTFServer::sendQuaternion(std::string frame, std::string parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, ros::Time rostime)
{
  sendQuaternion(lookup(frame), lookup(parent), xt, yt, zt, xr, yr, zr, w, rostime.sec, rostime.nsec);
};

void rosTFServer::sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs)
{
  if (!checkInvalidFrame(frame))
    return;
  
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

bool rosTFServer::checkInvalidFrame(unsigned int frameID)
{
  if (frameID == 0)
    {
      std::cerr<<"Attempted to send frameID = 0, this is not valid!" << std::endl;
      return false;
    }
  return true;
};

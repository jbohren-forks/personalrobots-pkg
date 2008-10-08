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
			 uint64_t max_cache_time,
			 uint64_t max_extrapolation_distance):
  TransformReference(interpolating,
                     max_cache_time,
                     max_extrapolation_distance),
  myNode(rosnode)
{
  //  printf("Constructed rosTF\n");
  myNode.subscribe("TransformArray", tfArrayIn, &rosTFClient::receiveArray, this,100);
};

rosTFClient::~rosTFClient()
{
  myNode.unsubscribe("TransformArray");

};

void rosTFClient::transformPointCloud(const std::string & target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::PointCloudFloat32 & cloudIn)
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

std_msgs::PointCloudFloat32 rosTFClient::transformPointCloud(const std::string & target_frame,  const std_msgs::PointCloudFloat32 & cloudIn)
{
    std_msgs::PointCloudFloat32 cloudOut;
    transformPointCloud(target_frame, cloudOut, cloudIn);
    return cloudOut;    
};



void rosTFClient::transformLaserScanToPointCloud(const std::string & target_frame, std_msgs::PointCloudFloat32 & cloudOut, const std_msgs::LaserScan & scanIn)
{
  cloudOut.header = scanIn.header;
  cloudOut.header.frame_id = target_frame;
  cloudOut.set_pts_size(scanIn.get_ranges_size());
    if (scanIn.get_intensities_size() > 0)
      {
        cloudOut.set_chan_size(1);
        cloudOut.chan[0].name ="intensities";
        cloudOut.chan[0].set_vals_size(scanIn.get_intensities_size());
      }

  libTF::TFPoint pointIn;
  libTF::TFPoint pointOut;

  pointIn.frame = scanIn.header.frame_id;
  
  ///\todo this can be optimized
  std_msgs::PointCloudFloat32 intermediate; //optimize out
  projector_.projectLaser(scanIn, intermediate, -1.0, true);
  
  unsigned int count = 0;  
  for (unsigned int i = 0; i < scanIn.get_ranges_size(); i++)
  {
    if (scanIn.ranges[i] < scanIn.range_max 
        && scanIn.ranges[i] > scanIn.range_min) //only when valid
    {
      pointIn.time = scanIn.header.stamp.to_ull() + (uint64_t) (scanIn.time_increment * 1000000000);
      pointIn.x = intermediate.pts[i].x;
      pointIn.y = intermediate.pts[i].y;
      pointIn.z = intermediate.pts[i].z;
      
      pointOut = transformPoint(target_frame, pointIn);
      
      cloudOut.pts[count].x  = pointOut.x;
      cloudOut.pts[count].y  = pointOut.y;
      cloudOut.pts[count].z  = pointOut.z;
      
      if (scanIn.get_intensities_size() >= i) /// \todo optimize and catch length difference better
	  cloudOut.chan[0].vals[count] = scanIn.intensities[i];
      count++;
    }
    
  }
  //downsize if necessary
  cloudOut.set_pts_size(count);
  cloudOut.chan[0].set_vals_size(count);
    
}



void rosTFClient::receiveArray()
{
  for (unsigned int i = 0; i < tfArrayIn.get_eulers_size(); i++)
  {
    try{
      setWithEulers(tfArrayIn.eulers[i].header.frame_id, tfArrayIn.eulers[i].parent, tfArrayIn.eulers[i].x, tfArrayIn.eulers[i].y, tfArrayIn.eulers[i].z, tfArrayIn.eulers[i].yaw, tfArrayIn.eulers[i].pitch, tfArrayIn.eulers[i].roll, tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + tfArrayIn.eulers[i].header.stamp.nsec);
    }    
    catch (libTF::Exception &ex)
    {
      std::cerr << "receiveArray: setWithEulers failed with frame_id "<< tfArrayIn.eulers[i].header.frame_id << " parent " << tfArrayIn.eulers[i].parent << std::endl;
      std::cerr<< ex.what();
    };
  }
  //std::cout << "received euler frame: " << tfArrayIn.eulers[i].header.frame_id << " with parent:" << tfArrayIn.eulers[i].parent << "time " << tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + eulerIn.header.stamp.nsec << std::endl;
  for (unsigned int i = 0; i < tfArrayIn.get_dhparams_size(); i++)
  {
    try{
    setWithDH(tfArrayIn.dhparams[i].header.frame_id, tfArrayIn.dhparams[i].parent, tfArrayIn.dhparams[i].length, tfArrayIn.dhparams[i].twist, tfArrayIn.dhparams[i].offset, tfArrayIn.dhparams[i].angle, tfArrayIn.dhparams[i].header.stamp.sec * 1000000000ULL + tfArrayIn.dhparams[i].header.stamp.nsec);
    }    
    catch (libTF::Exception &ex)
    {
      std::cerr << "receiveArray: setWithDH failed with frame_id "<< tfArrayIn.dhparams[i].header.frame_id << " parent " << tfArrayIn.dhparams[i].parent << std::endl;
      std::cerr<< ex.what();
    };
    //  std::cout << "recieved DH frame: " << tfArrayIn.dhparams[i].header.frame_id << " with parent:" << tfArrayIn.dhparams[i].parent << std::endl;
  }
  
  for (unsigned int i = 0; i < tfArrayIn.get_quaternions_size(); i++)
  {
    try{
    setWithQuaternion(tfArrayIn.quaternions[i].header.frame_id, tfArrayIn.quaternions[i].parent, tfArrayIn.quaternions[i].xt, tfArrayIn.quaternions[i].yt, tfArrayIn.quaternions[i].zt, tfArrayIn.quaternions[i].xr, tfArrayIn.quaternions[i].yr, tfArrayIn.quaternions[i].zr, tfArrayIn.quaternions[i].w, tfArrayIn.quaternions[i].header.stamp.sec * 1000000000ULL + tfArrayIn.quaternions[i].header.stamp.nsec);
    }    
    catch (libTF::Exception &ex)
    {
      std::cerr << "receiveArray: setWithQuaternion failed with frame_id "<< tfArrayIn.quaternions[i].header.frame_id << " parent " << tfArrayIn.quaternions[i].parent << std::endl;
      std::cerr<< ex.what();
    };
    //  std::cout << "recieved quaternion frame: " << tfArrayIn.quaternions[i].header.frame_id << " with parent:" << tfArrayIn.quaternions[i].parent << std::endl;
  }
  for (unsigned int i = 0; i < tfArrayIn.get_matrices_size(); i++)
  {
      
    try{
    if (tfArrayIn.matrices[i].get_matrix_size() != 16)
    {
      std::cerr << "recieved matrix not of size 16, it was "<< tfArrayIn.matrices[i].get_matrix_size();
      return;
    }
      
    NEWMAT::Matrix tempMatrix(4,4);
    tempMatrix << (tfArrayIn.matrices[i].matrix[0]);
    
    setWithMatrix(tfArrayIn.matrices[i].header.frame_id, tfArrayIn.matrices[i].parent, tempMatrix, tfArrayIn.matrices[i].header.stamp.sec * 1000000000ULL + tfArrayIn.matrices[i].header.stamp.nsec);
    
    }    
    catch (libTF::Exception &ex)
    {
      std::cerr << "receiveArray: setWithMatrix failed with frame_id "<< tfArrayIn.matrices[i].header.frame_id << " parent " << tfArrayIn.matrices[i].parent << std::endl;
      std::cerr<< ex.what();
    };
    //  std::cout << "recieved Matrix:" << tempMatrix << " in frame: " << tfArrayIn.matrices[i].header.header.frame_id << " with parent:" << tfArrayIn.matrices[i].parent << " at time:" << tfArrayIn.matrices[i].header.stamp.to_double() << std::endl;
  }

};

NEWMAT::Matrix rosTFClient::getMatrix(const std::string & target_frame, const std::string & source_frame, ros::Time time)
{ return getMatrix(target_frame, source_frame, time.to_ull());};


rosTFServer::rosTFServer(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.advertise<rosTF::TransformArray>("TransformArray", 100);
};

void rosTFServer::sendEuler(const std::string & frame, const std::string & parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime)
{

  rosTF::TransformArray tfArray;
  tfArray.set_eulers_size(1);

  tfArray.eulers[0].header.frame_id = frame;
  tfArray.eulers[0].parent = parent;
  tfArray.eulers[0].x = x;
  tfArray.eulers[0].y = y;
  tfArray.eulers[0].z = z;
  tfArray.eulers[0].yaw = yaw;
  tfArray.eulers[0].pitch = pitch;
  tfArray.eulers[0].roll = roll;
  tfArray.eulers[0].header.stamp = rostime;

  myNode.publish("TransformArray", tfArray);

};

void rosTFServer::sendInverseEuler(const std::string & frame, const std::string & parent, double x, double y, double z, double yaw, double pitch, double roll, ros::Time rostime)
{ 
  rosTF::TransformArray tfArray;
  tfArray.set_eulers_size(1);
  //Invert the transform
  libTF::Euler odomeuler  = libTF::Pose3D::eulerFromMatrix(libTF::Pose3D::matrixFromEuler(x, y, z, yaw, pitch, roll).i()); //todo optimize
  libTF::Position odompos = libTF::Pose3D::positionFromMatrix(libTF::Pose3D::matrixFromEuler(x, y, z, yaw, pitch, roll).i());
  
  tfArray.eulers[0].header.frame_id = frame;
  tfArray.eulers[0].parent = parent;
  tfArray.eulers[0].x = odompos.x;
  tfArray.eulers[0].y = odompos.y;
  tfArray.eulers[0].z = odompos.z;
  tfArray.eulers[0].yaw = odomeuler.yaw;
  tfArray.eulers[0].pitch = odomeuler.pitch;
  tfArray.eulers[0].roll = odomeuler.roll;
  tfArray.eulers[0].header.stamp = rostime;

  myNode.publish("TransformArray", tfArray);
 
};


void rosTFServer::sendPose(libTF::TFPose pose, const std::string & parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendEuler(pose.frame, parent, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, ros::Time(secs, nsecs));  
};

void rosTFServer::sendInversePose(libTF::TFPose pose, const std::string & parent)
{
  unsigned int nsecs = pose.time % 1000000000;
  unsigned int secs = (pose.time - nsecs) / 1000000000;
  sendInverseEuler(pose.frame, parent, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, ros::Time(secs, nsecs));  
};


void rosTFServer::sendDH(const std::string & frame, const std::string & parent, double length, double twist, double offset, double angle, ros::Time rostime)
{
  rosTF::TransformArray tfArray;
  tfArray.set_dhparams_size(1);

  tfArray.dhparams[0].header.frame_id = frame;
  tfArray.dhparams[0].parent = parent;
  tfArray.dhparams[0].length = length;
  tfArray.dhparams[0].twist = twist;
  tfArray.dhparams[0].offset = offset;
  tfArray.dhparams[0].angle = angle;
  tfArray.dhparams[0].header.stamp = rostime;

  myNode.publish("TransformArray", tfArray);

};

void rosTFServer::sendQuaternion(const std::string & frame, const std::string & parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, ros::Time rostime)
{ 
  rosTF::TransformArray tfArray;
  tfArray.set_quaternions_size(1);

  tfArray.quaternions[0].header.frame_id = frame;
  tfArray.quaternions[0].parent = parent;
  tfArray.quaternions[0].xt = xt;
  tfArray.quaternions[0].yt = yt;
  tfArray.quaternions[0].zt = zt;
  tfArray.quaternions[0].xr = xr;
  tfArray.quaternions[0].yr = yr;
  tfArray.quaternions[0].zr = zr;
  tfArray.quaternions[0].w = w;
  tfArray.quaternions[0].header.stamp = rostime;

  myNode.publish("TransformArray", tfArray);

};


void rosTFServer::sendMatrix(const std::string & frame, const std::string & parent, NEWMAT::Matrix matrix, ros::Time rostime)
{
  rosTF::TransformArray tfArray;
  tfArray.set_matrices_size(1);


  tfArray.matrices[0].header.frame_id = frame;
  tfArray.matrices[0].header.stamp = rostime;
  tfArray.matrices[0].parent = parent;

  tfArray.matrices[0].set_matrix_size(16);
  if (matrix.Nrows() != 4 || matrix.Ncols() != 4)
    return;  
  
  double * matPtr = matrix.Store();
  for (unsigned int i = 0; i < 16; i++)
  {
    tfArray.matrices[0].matrix[i] = matPtr[i];
  }

  myNode.publish("TransformArray", tfArray);

  
};


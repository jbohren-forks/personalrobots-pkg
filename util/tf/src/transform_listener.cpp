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

/** \author Tully Foote */

#include "tf/transform_listener.h"

using namespace tf;

void TransformListener::transformQuaternion(const std::string& target_frame,
                                            const std_msgs::QuaternionStamped& msg_in,
                                            std_msgs::QuaternionStamped& msg_out)
{
  Stamped<Quaternion> pin, pout;
  QuaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, pin, pout);
  QuaternionStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformVector(const std::string& target_frame,
                                        const std_msgs::Vector3Stamped& msg_in,
                                        std_msgs::Vector3Stamped& msg_out)
{
  Stamped<Vector3> pin, pout;
  Vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, pin, pout);
  Vector3StampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPoint(const std::string& target_frame,
                                       const std_msgs::PointStamped& msg_in,
                                       std_msgs::PointStamped& msg_out)
{
  Stamped<Point> pin, pout;
  PointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, pin, pout);
  PointStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPose(const std::string& target_frame,
                                      const std_msgs::PoseStamped& msg_in,
                                      std_msgs::PoseStamped& msg_out)
{
  Stamped<Pose> pin, pout;
  PoseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, pin, pout);
  PoseStampedTFToMsg(pout, msg_out);
}
void TransformListener::transformQuaternion(const std::string& target_frame, const ros::Time& target_time, 
                                            const std_msgs::QuaternionStamped& msg_in, 
                                            const std::string& fixed_frame, std_msgs::QuaternionStamped& msg_out)
{
  Stamped<Quaternion> pin, pout;
  QuaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, pin, pout);
  QuaternionStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformVector(const std::string& target_frame, const ros::Time& target_time, 
                                            const std_msgs::Vector3Stamped& msg_in, 
                                            const std::string& fixed_frame, std_msgs::Vector3Stamped& msg_out)
{
  Stamped<Vector3> pin, pout;
  Vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, pin, pout);
  Vector3StampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPoint(const std::string& target_frame, const ros::Time& target_time, 
                                            const std_msgs::PointStamped& msg_in, 
                                            const std::string& fixed_frame, std_msgs::PointStamped& msg_out)
{
  Stamped<Point> pin, pout;
  PointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, pin, pout);
  PointStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPose(const std::string& target_frame, const ros::Time& target_time, 
                                      const std_msgs::PoseStamped& msg_in, 
                                      const std::string& fixed_frame, std_msgs::PoseStamped& msg_out)
{
  Stamped<Pose> pin, pout;
  PoseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, pin, pout);
  PoseStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPointCloud(const std::string & target_frame, const std_msgs::PointCloud & cloudIn, std_msgs::PointCloud & cloudOut)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, cloudIn.header.frame_id, cloudIn.header.stamp, transform);

  transformPointCloud(target_frame, transform, cloudIn.header.stamp, cloudIn, cloudOut);
}
void TransformListener::transformPointCloud(const std::string& target_frame, const ros::Time& target_time, 
                                            const std_msgs::PointCloud& cloudIn, 
                                            const std::string& fixed_frame, std_msgs::PointCloud& cloudOut)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time, 
                  cloudIn.header.frame_id, cloudIn.header.stamp,
                  fixed_frame, 
                  transform);

  transformPointCloud(target_frame, transform, target_time, cloudIn, cloudOut);
  

}


void TransformListener::transformPointCloud(const std::string & target_frame, const Transform& net_transform, const ros::Time& target_time, const std_msgs::PointCloud & cloudIn, std_msgs::PointCloud & cloudOut)
{
  NEWMAT::Matrix transform = transformAsMatrix(net_transform);

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
  cloudOut.header.stamp = target_time;
  cloudOut.header.frame_id = target_frame;
  for (unsigned int i = 0; i < length ; i++) 
    { 
      cloudOut.pts[i].x = matrixPtr[i];
      cloudOut.pts[i].y = matrixPtr[1*length + i];
      cloudOut.pts[i].z = matrixPtr[2*length + i];
    };
}


void TransformListener::transformLaserScanToPointCloud(const std::string & target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::LaserScan & scanIn)
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

  tf::Stamped<tf::Point> pointIn;
  tf::Stamped<tf::Point> pointOut;

  pointIn.frame_id_ = scanIn.header.frame_id;
  
  ///\todo this can be optimized
  std_msgs::PointCloud intermediate; //optimize out
  projector_.projectLaser(scanIn, intermediate, -1.0, true);
  
  // Extract transforms for the beginning and end of the laser scan
  ros::Time start_time = scanIn.header.stamp ;
  ros::Time end_time   = scanIn.header.stamp + ros::Duration().fromSec(scanIn.get_ranges_size()*scanIn.time_increment) ;
  
  tf::Stamped<tf::Transform> start_transform ;
  tf::Stamped<tf::Transform> end_transform ;
  tf::Stamped<tf::Transform> cur_transform ;
  
  lookupTransform(target_frame, scanIn.header.frame_id, start_time, start_transform) ;
  lookupTransform(target_frame, scanIn.header.frame_id, end_time, end_transform) ;
  
  
  unsigned int count = 0;  
  for (unsigned int i = 0; i < scanIn.get_ranges_size(); i++)
  {
    if (scanIn.ranges[i] < scanIn.range_max 
        && scanIn.ranges[i] > scanIn.range_min) //only when valid
    {
      // Looking up transforms in tree is too expensive. Need more optimized way
      /*
      pointIn = tf::Stamped<tf::Point>(btVector3(intermediate.pts[i].x, intermediate.pts[i].y, intermediate.pts[i].z), 
                                       ros::Time(scanIn.header.stamp.to_ull() + (uint64_t) (scanIn.time_increment * 1000000000)),
                                       pointIn.frame_id_ = scanIn.header.frame_id);///\todo optimize to no copy
      transformPoint(target_frame, pointIn, pointOut);
      */

      // Instead, assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      btScalar ratio = i / ( (double) scanIn.get_ranges_size() - 1.0) ;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)
      
      //Interpolate translation
      btVector3 v ;
      v.setInterpolate3(start_transform.getOrigin(), end_transform.getOrigin(), ratio) ;
      cur_transform.setOrigin(v) ;
      
      //Interpolate rotation
      btQuaternion q1, q2 ;
      start_transform.getBasis().getRotation(q1) ;
      end_transform.getBasis().getRotation(q2) ;

      // Compute the slerp-ed rotation
      cur_transform.setRotation( slerp( q1, q2 , ratio) ) ;
      
      // Apply the transform to the current point
      btVector3 pointIn(intermediate.pts[i].x, intermediate.pts[i].y, intermediate.pts[i].z) ;
      btVector3 pointOut = cur_transform * pointIn ;
      
      // Copy transformed point into cloud
      cloudOut.pts[count].x  = pointOut.x();
      cloudOut.pts[count].y  = pointOut.y();
      cloudOut.pts[count].z  = pointOut.z();
      
      if (scanIn.get_intensities_size() >= i) /// \todo optimize and catch length difference better
	  cloudOut.chan[0].vals[count] = scanIn.intensities[i];
      count++;
    }
    
  }
  //downsize if necessary
  cloudOut.set_pts_size(count);
  cloudOut.chan[0].set_vals_size(count);
}


void TransformListener::subscription_callback()
{
  for (uint i = 0; i < msg_in_.transforms.size(); i++)
  {
    Stamped<Transform> trans;
    TransformStampedMsgToTF(msg_in_.transforms[i], trans);
    try 
    {
      setTransform(trans);
    }
    
    catch (TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform %s to %s with error: %s\n", msg_in_.transforms[i].header.frame_id.c_str(), msg_in_.transforms[i].parent_id.c_str(), temp.c_str());
    }
  }



};

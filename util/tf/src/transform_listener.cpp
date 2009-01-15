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
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);

  unsigned int length = cloudIn.get_pts_size();

  boost::numeric::ublas::matrix<double> matIn(4, length);

  //  double * matrixPtr = matIn.Store();

  for (unsigned int i = 0; i < length ; i++)
  {
    matIn(0,i) = cloudIn.pts[i].x;
    matIn(1,i) = cloudIn.pts[i].y;
    matIn(2,i) = cloudIn.pts[i].z;
    matIn(3,i) = 1;
  };

  boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);

  // Copy relevant data from cloudIn, if needed
  if (&cloudIn != &cloudOut)
  {
    cloudOut.header = cloudIn.header;
    cloudOut.set_pts_size(length);
    cloudOut.set_chan_size(cloudIn.get_chan_size());
    for (unsigned int i = 0 ; i < cloudIn.get_chan_size() ; ++i)
      cloudOut.chan[i] = cloudIn.chan[i];
  }

  //Override the positions
  cloudOut.header.stamp = target_time;
  cloudOut.header.frame_id = target_frame;
  for (unsigned int i = 0; i < length ; i++)
  {
    cloudOut.pts[i].x = matOut(0,i);
    cloudOut.pts[i].y = matOut(1,i);
    cloudOut.pts[i].z = matOut(2,i);
  };
}




void TransformListener::subscription_callback()
{
  for (unsigned int i = 0; i < msg_in_.transforms.size(); i++)
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


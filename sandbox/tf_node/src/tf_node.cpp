/*
 * Copyright (C) 2008, Jason Wolfe and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include <map>

#include "ros/ros.h"

#include "tf/tf.h"
#include "tf/exceptions.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

#include "tf/FrameGraph.h"

#include "robot_msgs/PointStamped.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/QuaternionStamped.h"
#include "robot_msgs/Vector3Stamped.h"

#include "tf_node/StartTransforming.h"
#include "tf_node/StopTransforming.h"

#include "tf_node/TransformPoint.h"
#include "tf_node/TransformPointCloud.h"
#include "tf_node/TransformPose.h"
#include "tf_node/TransformQuaternion.h"
#include "tf_node/TransformVector.h"


using namespace tf_node;

tf::TransformListener *tl;
tf::Transformer       *trf;
ros::NodeHandle       *nh;


bool getFramesCallback(tf::FrameGraph::Request &req, 
		       tf::FrameGraph::Response &res) {
  return tl->getFrames(req, res);
}

bool transformPointCallback(TransformPoint::Request &req, 
			    TransformPoint::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPoint(req.target_frame, req.pin, res.pout);
    } else {
      tl->transformPoint(req.target_frame, req.target_time, req.pin, req.fixed_frame, res.pout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformPointCloudCallback(TransformPointCloud::Request &req, 
			         TransformPointCloud::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPointCloud(req.target_frame, req.pcin, res.pcout);
    } else {
      tl->transformPointCloud(req.target_frame, req.target_time, req.pcin, req.fixed_frame, res.pcout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformPoseCallback(TransformPose::Request &req, 
			    TransformPose::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPose(req.target_frame, req.pin, res.pout);
    } else {
      tl->transformPose(req.target_frame, req.target_time, req.pin, req.fixed_frame, res.pout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformQuaternionCallback(TransformQuaternion::Request &req, 
			    TransformQuaternion::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformQuaternion(req.target_frame, req.qin, res.qout);
    } else {
       tl->transformQuaternion(req.target_frame, req.target_time, req.qin, req.fixed_frame, res.qout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformVectorCallback(TransformVector::Request &req, 
			    TransformVector::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformVector(req.target_frame, req.vin, res.vout);
    } else {
      tl->transformVector(req.target_frame, req.target_time, req.vin, req.fixed_frame, res.vout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}


//template<class MessageT>
//void doTransform(const std::string &target_frame, MessageT &in, MessageT &out);

void doTransform(const std::string &target_frame, const robot_msgs::PointStamped &in, robot_msgs::PointStamped &out) {
  tl->transformPoint(target_frame, in, out);
}

void doTransform(const std::string &target_frame, const robot_msgs::PointCloud &in, robot_msgs::PointCloud &out) {
  tl->transformPointCloud(target_frame, in, out);
}

void doTransform(const std::string &target_frame, const robot_msgs::PoseStamped &in, robot_msgs::PoseStamped &out) {
  tl->transformPose(target_frame, in, out);
}

void doTransform(const std::string &target_frame, const robot_msgs::QuaternionStamped &in, robot_msgs::QuaternionStamped &out) {
  tl->transformQuaternion(target_frame, in, out);
}

void doTransform(const std::string &target_frame, const robot_msgs::Vector3Stamped &in, robot_msgs::Vector3Stamped &out) {
  tl->transformVector(target_frame, in, out);
}


class GenericTopicTransformer {
public:
  GenericTopicTransformer() {}
  virtual ~GenericTopicTransformer() {}
};

template<class MessageT>
class TopicTransformer : public GenericTopicTransformer {
public:
  TopicTransformer(const std::string &in_topic, 
		   const std::string &out_topic,  
		   const std::string &target_frame,
		   uint32_t queue_size) 
    : _target_frame(target_frame),
      _pub      (nh->advertise<MessageT>(out_topic, queue_size)),
      _notifier (*trf, boost::bind(&TopicTransformer<MessageT>::callback, this, _1), 
		 in_topic, target_frame, queue_size),
      _out      ()
  {
    //    ros::Duration d(1,0);
    //_notifier.setTolerance(d);
  }

private:
  void callback(const boost::shared_ptr<const MessageT> message) {
    ROS_INFO("callback");
      try {
         doTransform(_target_frame, *message, _out);
	 _pub.publish(_out);
      } catch (tf::TransformException &e) {
         ROS_ERROR(e.what());
      }
  }

  const std::string _target_frame;
  ros::Publisher _pub;
  tf::MessageNotifier<MessageT> _notifier;
  MessageT _out;
  
};

std::map<std::string, boost::shared_ptr<GenericTopicTransformer> > transformers;


void startTransformingCallback(const boost::shared_ptr<StartTransforming const> &msg) {
  if (transformers.find(msg->out_topic) != transformers.end()) {
    ROS_ERROR((msg->out_topic + " is already being published.").c_str());
    return;
  }
  switch (msg->type) {

  case StartTransforming::point:
    transformers[msg->out_topic] = boost::shared_ptr<GenericTopicTransformer>(new TopicTransformer<robot_msgs::PointStamped>(msg->in_topic, msg->out_topic, msg->target_frame, msg->queue_size));
    break;

  case StartTransforming::point_cloud:
    transformers[msg->out_topic] = boost::shared_ptr<GenericTopicTransformer>(new TopicTransformer<robot_msgs::PointCloud>(msg->in_topic, msg->out_topic, msg->target_frame, msg->queue_size));   
    break;

  case StartTransforming::pose:
    transformers[msg->out_topic] = boost::shared_ptr<GenericTopicTransformer>(new TopicTransformer<robot_msgs::PoseStamped>(msg->in_topic, msg->out_topic, msg->target_frame, msg->queue_size));
    break;

  case StartTransforming::vector:
    transformers[msg->out_topic] = boost::shared_ptr<GenericTopicTransformer>(new TopicTransformer<robot_msgs::Vector3Stamped>(msg->in_topic, msg->out_topic, msg->target_frame, msg->queue_size));	    
    break;

  case StartTransforming::quaternion:
    transformers[msg->out_topic] = boost::shared_ptr<GenericTopicTransformer>(new TopicTransformer<robot_msgs::QuaternionStamped>(msg->in_topic, msg->out_topic, msg->target_frame, msg->queue_size));
    break;

  default:
    ROS_ERROR("Unknown message type");
    return;
  }
  ROS_INFO((msg->in_topic + " will now be transformed into " + msg->out_topic).c_str());
}

void stopTransformingCallback(const boost::shared_ptr<StopTransforming const> &msg) {
  int n_removed = transformers.erase(msg->out_topic);
  if (n_removed == 0) {
    ROS_ERROR((msg->out_topic + " is not a currently published transformer.").c_str());
  } else {
    ROS_INFO((msg->out_topic + " will no longer be published.").c_str());
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_node");

  nh = new ros::NodeHandle();

  // Use default arguments
  tl = new tf::TransformListener();
  trf = new tf::Transformer();

  ros::ServiceServer f  = nh->advertiseService("get_tf_frames",         &getFramesCallback);
  ros::ServiceServer p  = nh->advertiseService("transform_point",       &transformPointCallback);
  ros::ServiceServer pc = nh->advertiseService("transform_point_cloud", &transformPointCloudCallback);
  ros::ServiceServer ps = nh->advertiseService("transform_pose",        &transformPoseCallback);
  ros::ServiceServer q  = nh->advertiseService("transform_quaternion",  &transformQuaternionCallback);
  ros::ServiceServer v  = nh->advertiseService("transform_vector",      &transformVectorCallback);

  // These don't seem to be working right now ...
  ros::Subscriber startTransformer = nh->subscribe("start_transforming", 20, &startTransformingCallback);
  ros::Subscriber stopTransforming = nh->subscribe("stop_transforming", 20,  &stopTransformingCallback);

  ros::spin();

  delete nh;
  delete tl;
  delete trf;

  return 0;
}

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
#include "tf/transform_broadcaster.h"
#include "tf/message_notifier.h"

#include "tf/FrameGraph.h"

#include "tf/tfMessage.h"


#include "robot_msgs/PointStamped.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/QuaternionStamped.h"
#include "robot_msgs/Vector3Stamped.h"

#include "robot_msgs/Vector3.h"
#include "robot_msgs/Quaternion.h"
#include "robot_msgs/Transform.h"
#include "robot_msgs/TransformStamped.h"


#include "tf_node/TransformPoint.h"
#include "tf_node/TransformPointCloud.h"
#include "tf_node/TransformPose.h"
#include "tf_node/TransformQuaternion.h"
#include "tf_node/TransformVector.h"

#include "tf_node/StartStaticTransform.h"
#include "tf_node/StopStaticTransform.h"

#include "tf_node/StartTransforming.h"
#include "tf_node/StopTransforming.h"


using namespace tf_node;

tf::TransformListener    *tl;
tf::TransformBroadcaster *tb;
tf::Transformer          *trf;
ros::NodeHandle          *nh;


/******************************************************************************
 *  TransformListener methods
 ******************************************************************************/

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



/******************************************************************************
 *  Publishing (static) transforms
 ******************************************************************************/

void publishTransformCallback(const boost::shared_ptr<const robot_msgs::TransformStamped> &message) {
  tf::Transform tft;
  tf::transformMsgToTF(message->transform, tft);
  ROS_INFO((std::string("Sending transform from parent ") + message->parent_id + " to " + message->header.frame_id).c_str());
  tb->sendTransform(tft, message->header.stamp, message->header.frame_id, message->parent_id);
}

class TimedTransform {
public:
  TimedTransform(const boost::shared_ptr<const StartStaticTransform> &transform) : 
	_transform(transform), _timer(nh->createTimer(transform->frequency, &TimedTransform::callback, this)) {
    tf::transformMsgToTF(transform->transform, _tft);
  }

private:
  void callback(const ros::TimerEvent &te) {
    ROS_INFO((std::string("Sending static transform from parent ") + _transform->parent_id + " to " + _transform->frame_id).c_str());
    tb->sendTransform(_tft, ros::Time::now() + _transform->frequency + _transform->frequency, 
	_transform->frame_id, _transform->parent_id);
  }

  boost::shared_ptr<const StartStaticTransform> _transform;
  tf::Transform _tft;
  ros::Timer _timer;
};

std::map<std::pair<std::string, std::string>, boost::shared_ptr<TimedTransform> > static_transforms;
boost::mutex static_transform_mutex;

void startStaticTransformCallback(const boost::shared_ptr<const StartStaticTransform> &message) {
  boost::mutex::scoped_lock l(static_transform_mutex);
  if (static_transforms.erase(std::pair<std::string, std::string>(message->frame_id, message->parent_id)) > 0) {
    ROS_INFO((std::string("Supplanting existing static transform from parent ") + message->parent_id + " to " + message->frame_id).c_str());
  }
  static_transforms[std::pair<std::string, std::string>(message->frame_id, message->parent_id)]
    = boost::shared_ptr<TimedTransform>(new TimedTransform(message));
  ROS_INFO((std::string("Added static transform from parent ") + message->parent_id + " to " + message->frame_id).c_str());
}

void stopStaticTransformCallback(const boost::shared_ptr<const StopStaticTransform> &message) {
  boost::mutex::scoped_lock l(static_transform_mutex);
  if (static_transforms.erase(std::pair<std::string, std::string>(message->frame_id, message->parent_id)) > 0) {
    ROS_INFO((std::string("Stopped static transform from parent ") + message->parent_id + " to " + message->frame_id).c_str());
  } else { 
    ROS_INFO((std::string("Couldn't stop non-existant static transform from parent ") + message->parent_id + " to " + message->frame_id).c_str());
  }
}


/******************************************************************************
 *  Setting up standing transforms
 ******************************************************************************/


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


/******************************************************************************
 *  main; set up services and topics
 ******************************************************************************/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_node");

  nh = new ros::NodeHandle();

  // Use default arguments
  tl = new tf::TransformListener();
  tb = new tf::TransformBroadcaster();
  trf = new tf::Transformer();

  // Services for transforming things
  ros::ServiceServer f  = nh->advertiseService("~get_tf_frames",         &getFramesCallback);
  ros::ServiceServer p  = nh->advertiseService("~transform_point",       &transformPointCallback);
  ros::ServiceServer pc = nh->advertiseService("~transform_point_cloud", &transformPointCloudCallback);
  ros::ServiceServer ps = nh->advertiseService("~transform_pose",        &transformPoseCallback);
  ros::ServiceServer q  = nh->advertiseService("~transform_quaternion",  &transformQuaternionCallback);
  ros::ServiceServer v  = nh->advertiseService("~transform_vector",      &transformVectorCallback);

  // Topics for publishing transforms  
  ros::Subscriber publishTransform     = 
     nh->subscribe<robot_msgs::TransformStamped>("~publish_transform", 20, &publishTransformCallback);
  ros::Subscriber startStaticTransform = 
     nh->subscribe<StartStaticTransform>("~start_static_transform", 20, &startStaticTransformCallback);
  ros::Subscriber stopStaticTransform  = 
     nh->subscribe<StopStaticTransform>("~stop_static_transform", 20, &stopStaticTransformCallback);

  // Topics for setting up streaming transforms
  // These don't seem to be working right now ...
  ros::Subscriber startTransformer = nh->subscribe("~start_transforming", 20, &startTransformingCallback);
  ros::Subscriber stopTransforming = nh->subscribe("~stop_transforming", 20,  &stopTransformingCallback);

  ros::spin();

  delete nh;
  delete tl;
  delete tb;
  delete trf;

  return 0;
}

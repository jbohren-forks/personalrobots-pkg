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

#include "tf/transform_listener.h"
#include "tf/message_notifier.h"
#include <boost/bind.hpp>
#include "ros/node.h"

void print_pose(const std::string& name, const tf::Stamped<tf::Pose>& pose)
{
  ROS_INFO("%s is at (%f %f %f) with orientation (%f, %f, %f, %f) in frame %s at time %.2f", 
           name.c_str(), 
           pose.getOrigin().getX(),
           pose.getOrigin().getY(),
           pose.getOrigin().getZ(),
           pose.getRotation().getX(),
           pose.getRotation().getY(),
           pose.getRotation().getZ(),
           pose.getRotation().getW(),
           pose.frame_id_.c_str(),
           pose.stamp_.toSec());

}

class NotifierUsingClass
{
public:
  tf::TransformListener& tfl_;

  void notifierCallback(const tf::MessageNotifier<geometry_msgs::PoseStamped>::MessagePtr& message)
  {
    tf::Stamped<tf::Pose> pose;
    tf::poseStampedMsgToTF(*message, pose);
    ROS_INFO("Notifer Callback");
    try{
    tfl_.transformPose("canaveral", pose, pose);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("This sould never be called for the notifier only callsback when this is possible.  But it's good practice to use it anyway.");
    print_pose("object", pose);    
    }    
  };
  
  NotifierUsingClass(tf::TransformListener& tfl): tfl_(tfl)
  {
    
  };
  
  
};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  ros::Node node("tf_tutorial_listener");

  tf::TransformListener tfl(ros::Duration(15.0));  //Create a TransformListener with 15 seconds of caching


  while(node.ok())
  {
    //initializing a pose that we'll use multiple times
    tf::Pose identity;
    identity.setIdentity();
    //the same as tf::Pose(tf::Quaternion(0,0,0), tf::Point(0,0,0))
    
    //Basic API
    ROS_INFO("Where is the shuttle in relation to hawaii right now?");
    tf::Stamped<tf::Pose> shuttle_pose(identity, ros::Time::now(), "space_shuttle");
    try{
      tfl.transformPose("hawaii", shuttle_pose, shuttle_pose);
      print_pose("Shuttle", shuttle_pose);  
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Couldn't transform at time now : %s", ex.what());
    }

    //Latest Available
    ROS_INFO("Lets try to get the latest transform, a timestamp of 0 becomes latest available");
    shuttle_pose.stamp_ = ros::Time().fromSec(0);
    try {
      tfl.transformPose("hawaii", shuttle_pose, shuttle_pose);
      print_pose("Shuttle", shuttle_pose);  
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Couldn't transform at latest: %s", ex.what());
    }
    
    
    //Time Traveling API
    ROS_INFO("The shuttle was observed 5 seconds ago, where was that location with respect hawaii as close to now as possible, assuming that the shuttle did not move in relationship to earth.");
    shuttle_pose.stamp_ = ros::Time::now() - ros::Duration(5);
    try {
      tfl.transformPose("hawaii", ros::Time(), shuttle_pose, "sun", shuttle_pose);
      print_pose("Shuttle", shuttle_pose);  
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Time Travel: Couldn't transform at latest: %s", ex.what());
    }
    
    
    //canTransform
    ROS_INFO("Can I transform between hawaii and the sun at time now?");
    if (tfl.canTransform("hawaii", "sun", ros::Time::now()))
      ROS_INFO("YES");
    else
      ROS_INFO("NO");
      

    //Notifier
    ROS_INFO("This class will buffer messages coming in on topic objects and provide a callback when they can be transformed into target frame canaveral");
    NotifierUsingClass notifier_callback_class(tfl);   
    tf::MessageNotifier<geometry_msgs::PoseStamped>* notifier = new tf::MessageNotifier<geometry_msgs::PoseStamped>(tfl,
                                                                                                              boost::bind(&NotifierUsingClass::notifierCallback, &notifier_callback_class, _1), //callback function
                                                                                                              std::string("object"), //topic
                                                                                                              std::string("canaveral"), //target_frame
                                                                                                              100); //queue size

          
    sleep(10);
    delete notifier;
    ROS_INFO("RESTARTING");
  }
  return 0;
};


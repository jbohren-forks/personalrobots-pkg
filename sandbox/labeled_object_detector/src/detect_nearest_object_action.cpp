/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <detect_nearest_object_action.h>
#include <labeled_object_detector/PoseStampedState.h>
#include <robot_actions/action_runner.h>


using namespace std;
using namespace robot_msgs;
using namespace labeled_object_detector;



DetectNearestObjectAction::DetectNearestObjectAction(const std::string& name):robot_actions::Action<robot_msgs::PoseStamped, robot_msgs::PoseStamped>(name)
{

  tf_ = boost::shared_ptr<tf::TransformListener>(new  tf::TransformListener);
  detector_.setTFListener(tf_);

  n_.param( std::string("~fixed_frame"), fixed_frame_, std::string("map"));

  marker_pub_=n_.advertise<visualization_msgs::Marker>("visualization_marker",1);
  cloud_pub_ = n_.advertise<PointCloud>("object_cloud",100);
  box_pub_ = n_.advertise<BoundingBox>("object_boxes",100);
}



void DetectNearestObjectAction::setup()
{
  detector_.setup();
}

robot_actions::ResultStatus DetectNearestObjectAction::execute(const PoseStamped& goal, PoseStamped& feedback) 
{
  ROS_INFO("Execute action");
  
  // Variable for accumulated running time
  double count = 0.0;
  ros::Duration d; d.fromSec(0.001);

  goal_ = goal;

  tf_->transformPose(fixed_frame_, goal_, goal_fixed_);

  cloud_sub_ = boost::shared_ptr<ros::Subscriber>(new ros::Subscriber(n_.subscribe<PointCloud>("cloud",100,boost::bind(&DetectNearestObjectAction::cloudCallback, this,  _1))));


  while (!isPreemptRequested() ) {
    count += 1.0;
    d.sleep();
  }

  cloud_sub_.reset();

  if (isPreemptRequested()) {
    return robot_actions::PREEMPTED;
  }
  else {
    return robot_actions::SUCCESS;
  }
}



Point32 pt_to_pt32(const Point& in)
{
  Point32 out;
  out.x=float(in.x);
  out.y=float(in.y);
  out.z=float(in.z);
  return out;
}


void DetectNearestObjectAction::cloudCallback( const PointCloudConstPtr& the_cloud)
{
  ROS_INFO("on cloud");
  boost::mutex::scoped_lock lock(proc_mutex_);
  try{
    ROS_INFO("Received point cloud, starting detection");
    cloud_ = the_cloud;

    ObjectModelDeque objects;
    detector_.detectObjects(*the_cloud,objects);
    ObjectModelDeque::iterator best_model=objects.end();
    double min_distance=1e15;

    for(ObjectModelDeque::iterator it=objects.begin();it!=objects.end(); it++)
    {
      best_model = it;
      PointStamped object_pose=(*it)->getCenter();
      PointStamped object_center;
      tf_->transformPoint(fixed_frame_, object_pose,object_center);

      double distance=cloud_geometry::distances::pointToPointXYDistanceSqr(pt_to_pt32(object_center.point),pt_to_pt32(goal_fixed_.pose.position));

      if(distance<min_distance)
      {
        min_distance=distance;
        best_model=it;
      }
    }

    if(best_model!=objects.end())
    {
      marker_pub_.publish((*best_model)->marker_);
      box_pub_.publish((*best_model)->bbox_);

      tf_->setTransform((*best_model)->object_frame_);
  
      broadcaster_.sendTransform((*best_model)->object_frame_);

      PlanarObjectModel* planar_model = dynamic_cast<PlanarObjectModel*>((*best_model).get());
      if(planar_model)
      {
        cloud_pub_.publish(planar_model->pcd_);
      }


      update((*best_model)->getPose());

    }
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }
}




/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "object_detector");

  DetectNearestObjectAction action("detect_nearest_object");

  action.setup();
  // Now run it.
  robot_actions::ActionRunner runner(10.0);
  runner.connect<PoseStamped, PoseStampedState, PoseStamped>(action);
  runner.run();

  ros::spin();

  return (0);
}
/* ]--- */


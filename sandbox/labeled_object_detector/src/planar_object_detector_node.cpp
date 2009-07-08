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


#include <planar_object_detector_node.h>
#include <pcd_misc.h>
#include <visualization_msgs/Marker.h>

#include "object_names/Name2Float.h"

using namespace std;
using namespace robot_msgs;
using namespace pcd_misc;
using namespace labeled_object_detector;



PlanarObjectDetectorNode::PlanarObjectDetectorNode()
{

  tf_ = boost::shared_ptr<tf::TransformListener>(new  tf::TransformListener);
  detector_.setTFListener(tf_);


  cloud_sub_ = n_.subscribe<PointCloud>("cloud",100,boost::bind(&PlanarObjectDetectorNode::cloudCallback, this, _1));

  marker_pub_=n_.advertise<visualization_msgs::Marker>("visualization_marker",1);
  cloud_pub_ = n_.advertise<PointCloud>("object_cloud",100);
  box_pub_ = n_.advertise<BoundingBox>("object_boxes",100);
}



void PlanarObjectDetectorNode::setup()
{
  detector_.setup();
}

void PlanarObjectDetectorNode::cloudCallback(const PointCloudConstPtr& the_cloud)
{
  boost::mutex::scoped_lock lock(proc_mutex_);
  try{
    ROS_INFO("Received point cloud, starting detection");
    cloud_ = the_cloud;

    ObjectModelDeque objects;
    detector_.detectObjects(*the_cloud,objects);
    for(ObjectModelDeque::iterator it=objects.begin();it!=objects.end(); it++)
    {
      marker_pub_.publish((*it)->marker_);
      box_pub_.publish((*it)->bbox_);

      tf_->setTransform((*it)->object_frame_);
  
      broadcaster_.sendTransform((*it)->object_frame_);

      PlanarObjectModel* planar_model = dynamic_cast<PlanarObjectModel*>((*it).get());
      if(planar_model)
      {
        cloud_pub_.publish(planar_model->pcd_);
      }
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


  PlanarObjectDetectorNode pod;

  pod.setup();


  ros::spin();

  return (0);
}
/* ]--- */


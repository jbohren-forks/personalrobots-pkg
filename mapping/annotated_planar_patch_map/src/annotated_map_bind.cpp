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

#include "ros/node.h"
#include "ros/ros.h"

// Services
#include "annotated_planar_patch_map/map_base_assembler_srv.h"

// Messages
//include "robot_msgs/PointCloud.h"
//include "pr2_mechanism_controllers/LaserScannerSignal.h"

using namespace robot_msgs ;

/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the scans/clouds between two tilt-laser shutters
 * params
 *  * "~target_frame_id" (string) - This is the frame that the scanned data transformed into.  The
 *                                  output clouds are also published in this frame.
 */


namespace annotated_planar_patch_map
{

class BindToObservation
{

public:

  tf::TransformListener *tf_;

  bool first_time_ ;

  std::string fixed_frame_ ;

  ros::Time start_time;

  ros::NodeHandle n_;
  ros::Publisher pub_;

  std::string query_;

  BindToObservation() 
  {
    pub_=n_.advertise<annotated_map_msgs::TaggedPolygonalMap> ("bound_map", 1) ;


    start_time = ros::Time::now();
    n_.param("~query", query_, std::string("*")) ;

    n_.param("~local_frame", query_, std::string("map")) ;

    tf_ = new tf::TransformListener( *this, true, ros::Duration(30.0));
  }

  ~AnnotatedMapQuerySnapshotter()
  {
  }

  void makeMap()
  {

    ROS_DEBUG("Printing map");

    QueryAnnotatedMap::Request map_req ;
    QueryAnnotatedMap::Response map_resp ;
    
    req.begin = ros::Time(0.0);//start_time;
    req.end   = ros::Time::now();
    req.query = query_;
    
    if (!ros::service::call("geom_global_map", req, resp))
    {
      ROS_ERROR("Failed to call query_annotated_map service.");
      return;
    } 


    QueryAnnotatedMap::Request local_map_req ;
    QueryAnnotatedMap::Response local_map_resp ;
    
    req.begin = ros::Time(0.0);//start_time;
    req.end   = ros::Time::now();
    req.query = query_;
    
    if (!ros::service::call("geom_local_map", local_map_req, local_map_resp))
    {
      ROS_ERROR("Failed to call query_local_map service.");
      return;
    } 

    annotated_map_msgs::TaggedPolygonalMap global_map_locally;
    annotated_map_lib::transformAnyObject(local_map_resp.map.frame_id,local_map_resp.map.stamp,tf_,local_map_resp.map,global_map_locally);
    
    transferAnnotations(global_map_locally,local_map_resp.map,0.1);
  
    ros::Node::instance()->publish("full_map", local_map_resp.map) ;
    ROS_DEBUG("Annotation binder::Published map size=%u", local_map_resp.map.get_polygons_size()) ;

  }
  void run()
  {

    while(1)
      {
	makeMap();
	//Wait a duration of one second.
	ros::Duration d = ros::Duration(1, 0);
	d.sleep();
	if(!n_.ok()){
	  break;
	}
      }
  }  
  
} ;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "am_snapshotter");
  annotated_planar_patch_map::AnnotatedMapQuerySnapshotter snapshotter ;
  //ros::spin();

  snapshotter.run();

  return 0;
}

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

class AnnotatedMapQuerySnapshotter
{

public:

  bool first_time_ ;

  std::string fixed_frame_ ;

  ros::Time start_time;

  ros::NodeHandle n_;
  ros::Publisher pub_;

  std::string query_;

  AnnotatedMapQuerySnapshotter() 
  {
    pub_=n_.advertise<annotated_map_msgs::TaggedPolygonalMap> ("full_map", 1) ;
    start_time = ros::Time::now();
    n_.param("~query", query_, std::string("*")) ;
  }

  ~AnnotatedMapQuerySnapshotter()
  {
  }

  void makeMap()
  {

    ROS_DEBUG("Printing map");

      QueryAnnotatedMap::Request req ;
      QueryAnnotatedMap::Response resp ;

      req.begin = ros::Time(0.0);//start_time;
      req.end   = ros::Time::now();
      req.query = query_;

      if (!ros::service::call("query_annotated_map", req, resp))
	{
	  ROS_ERROR("Failed to call query_annotated_map service.");
	  return;
	} 

      ros::Node::instance()->publish("full_map", resp.map) ;
      ROS_DEBUG("Snapshotter::Published map size=%u", resp.map.get_polygons_size()) ;

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

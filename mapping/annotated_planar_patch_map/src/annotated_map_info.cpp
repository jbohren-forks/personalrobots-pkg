/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*
* Author: Alexander Sorokin
*********************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include "ros/node.h"
#include "ros/publisher.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cv.h>


#include <robot_msgs/PolygonalMap.h>
#include <image_msgs/StereoInfo.h>

#include <cv_mech_turk/ExternalAnnotation.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include "annotated_planar_patch_map/annotated_map_lib.h"


#include "ros/ros.h"
#include "std_msgs/String.h"


class AnnotatedMapInfo
{
public:
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_;
  std::string actual_topic_;

  AnnotatedMapInfo(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
  {
  }

  void init()
  {
    sub_=node_handle_.subscribe<annotated_map_msgs::TaggedPolygonalMap>("annotated_map", 1000, &AnnotatedMapInfo::mapCallback, this);
    actual_topic_ = node_handle_.mapName("annotated_map");
  }

  void mapCallback(const annotated_map_msgs::TaggedPolygonalMapConstPtr& map)
  {
    ROS_INFO_STREAM("Map on " << actual_topic_);
    ROS_INFO("\t frame %s",map->header.frame_id.c_str() );
    ROS_INFO_STREAM("\t time "<< map->header.stamp );
    ROS_INFO("\t%d polygons",map->polygons.size());

    boost::unordered_map<std::string, int> map_tags=annotated_map_lib::getAllMapTags(*map);

    unsigned int num_tags=map_tags.size();
    ROS_INFO("\t%d tags",num_tags);
    boost::unordered_map<std::string, int>::iterator tag_iter=map_tags.begin();
    for(unsigned int iT=0;iT<num_tags;iT++)
      {
	//std::string s=tag_iter->first;
	ROS_INFO("\t\t%s %d",tag_iter->first.c_str(),tag_iter->second);
	tag_iter++;
      }
    double total_map_area = annotated_map_lib::getMapArea(*map);
    ROS_INFO("\tsurface area: %g square meters",total_map_area);

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "annotated_map_info");
  ros::NodeHandle n;

  AnnotatedMapInfo info(n);
  info.init();

  ros::spin();

  return 0;
}


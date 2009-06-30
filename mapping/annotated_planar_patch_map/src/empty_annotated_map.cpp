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
#include "ros/ros.h"

#include <cv.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <mapping_msgs/PolygonalMap.h>
#include <sensor_msgs/StereoInfo.h>

#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include <annotated_planar_patch_map/annotated_map_lib.h>

using namespace std;
using namespace tf;


class EmptyAnnotatedMap
{
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_;
  ros::Publisher pub_;


public:
  EmptyAnnotatedMap(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
  {
  }

  void init()
  {
    out_topic_name_=std::string("empty_poly_map");

    tf_ = new tf::TransformListener( *node_handle_.getNode(), true);

    node_handle_.param( std::string("~fixed_frame"), fixed_frame_, std::string("ERROR_NO_NAME")) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    ROS_ERROR("Need to set parameter fixed_frame") ;

    sub_=node_handle_.subscribe<mapping_msgs::PolygonalMap>("planar_map", 100, &EmptyAnnotatedMap::handleUnlabeledMap, this);

    pub_=node_handle_.advertise<annotated_map_msgs::TaggedPolygonalMap>(out_topic_name_,1);
  };


  void handleUnlabeledMap(const mapping_msgs::PolygonalMapConstPtr& unlabeled_map)
  {

    try
    {
      annotated_map_msgs::TaggedPolygonalMap polymapOut;

      mapping_msgs::PolygonalMap transformed_map_3D;
      annotated_map_lib::transformAnyObject(fixed_frame_,tf_,*unlabeled_map,transformed_map_3D);

      unsigned int num_polygons = transformed_map_3D.get_polygons_size();
      if(num_polygons==0)
        return;

      polymapOut.set_polygons_size(num_polygons);

      for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
      {
        //create new tagged polygon
        annotated_map_msgs::TaggedPolygon3D newPoly;
        newPoly.set_tags_size(1);
        newPoly.tags[0]="dup";
        newPoly.set_tags_chan_size(1);
        newPoly.tags_chan[0].name="hits";
        newPoly.tags_chan[0].set_vals_size(1);
        newPoly.tags_chan[0].vals[0]=1;
        newPoly.polygon=transformed_map_3D.polygons[iPoly];
	    
        //append polygon to the map
        polymapOut.polygons[iPoly]=newPoly;
      }
      polymapOut.header.frame_id=fixed_frame_;
      polymapOut.header.stamp=unlabeled_map->header.stamp;

      pub_.publish(polymapOut);     

    }
    catch (TransformException& ex)
    {
      ROS_ERROR("Failure to transform detected object:: %s\n", ex.what());
    }


  };

protected:
  tf::TransformListener *tf_;

  std::string fixed_frame_;

  std::string out_topic_name_;


  double dist_tolerance_; //in pixels
  double min_depth_; //in meters?
  double max_depth_; //in meters?
  int min_num_indist_tolerance_; //in vertices
  int max_allowed_num_outdist_tolerance_; //in vertices

};

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "empty_annotated_map");
    ros::NodeHandle n;
    
    EmptyAnnotatedMap map_converter(n);
    map_converter.init();

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}


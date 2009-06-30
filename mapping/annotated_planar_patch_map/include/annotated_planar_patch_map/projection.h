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

#ifndef ANNOTATED_MAP_PROJECTION_LIB_H
#define ANNOTATED_MAP_PROJECTION_LIB_H


#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/unordered_map.hpp>


#include <ros/common.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <robot_msgs/PolygonalMap.h>
#include <sensor_msgs/StereoInfo.h>
#include <sensor_msgs/CamInfo.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include <cv_mech_turk/ExternalAnnotation.h>

namespace annotated_planar_patch_map
{
namespace projection
{

void projectAnyObject(const sensor_msgs::CamInfo& cam_info,robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut);

void projectAnyObject(const sensor_msgs::StereoInfo& stereo_info_, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D);

void projectAnyObject(const sensor_msgs::CamInfo& stereo_info_, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D);





void projectPolygonalMap(const sensor_msgs::StereoInfo& stereo_info_, const robot_msgs::PolygonalMap& transformed_map_3D, robot_msgs::PolygonalMap &transformed_map_2D);

void projectPolygonPoints(double* projection,double img_w, double img_h, robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut);
void projectPolygonPointsNOP(double* projection,double img_w, double img_h, robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut);




void projectAnyObjectNOP(const sensor_msgs::CamInfo& stereo_info_, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D);

/* !
 * \brief Finds which polygons are visible in the current view. The polygons should be transformed 
 *        by the projection. The last coordinate should be depth for depth checking to work.
 *
 * @param map - the map, which polygons are checked. Should be projected into the camera frame.
p * @param viewport - [minX, maxX, minY, maxY, minZ, maxZ];
 */
std::vector<int> getVisibleProjectedPolygons(const annotated_map_msgs::TaggedPolygonalMap& map,
                                             const std::vector<double>& viewport); 


bool checkPolyInside(const robot_msgs::Polygon3D& poly,const std::vector<double>& viewport);

} 
} //end namespace


#endif

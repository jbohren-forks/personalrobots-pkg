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

#ifndef ANNOTATED_MAP_LIB_H
#define ANNOTATED_MAP_LIB_H


#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/unordered_map.hpp>


#include <ros/common.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <robot_msgs/PolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include <cv_mech_turk/ExternalAnnotation.h>

namespace annotated_map_lib
{

/* transforms */
//Note that this transform function has NO template version. 
//This will generate compile time error if you got no implementation.
void transformAnyObject(const std::string & target_frame, 
                        const tf::Transform* net_transform, 
                        const robot_msgs::PolygonalMap & polymap_in, 
                        robot_msgs::PolygonalMap & polymap_out);


void transformAnyObject(const std::string & target_frame, 
                        const tf::Transform* net_transform, 
                        const annotated_map_msgs::TaggedPolygonalMap & tagged_polymap_in, 
                        annotated_map_msgs::TaggedPolygonalMap & tagged_polymap_out);

void transformAnyObject(const std::string & target_frame, 
                        const tf::Transform* net_transform, 
                        const annotated_map_msgs::TaggedPolygon3D & poly_in, 
                        annotated_map_msgs::TaggedPolygon3D & poly_out);


boost::numeric::ublas::matrix<double> transformAsMatrix(const tf::Transform& bt);



template<class T>
void transformAnyObject(const std::string & target_frame, 
                        tf::TransformListener *tf_listener,
                        const T & object_in, 
                        T & object_out)
{
  tf::Stamped<tf::Transform> transform;
  tf_listener->lookupTransform(target_frame, object_in.header.frame_id, object_in.header.stamp, transform);
  
  annotated_map_lib::transformAnyObject(target_frame, (const tf::Transform*)&transform, object_in, object_out);
}


template<class T>
void transformAnyObject(const std::string& target_frame, 
                        const ros::Time& target_time,
                        tf::TransformListener *tf_listener,
                        const T& object_in,
                        const std::string& fixed_frame, 
                        T& object_out)
{
  tf::Stamped<tf::Transform> transform;
  tf_listener->lookupTransform(target_frame, target_time,
                               object_in.header.frame_id, object_in.header.stamp,
                               fixed_frame,
                               transform);
  
  annotated_map_lib::transformAnyObject(target_frame, (const tf::Transform*)&transform, object_in, object_out);
  
}


/* !
 * 
 * \brief Get all possible tags available in the map. Each tag is mapped into its count.
 *
 *
 */

boost::unordered_map<std::string, int> getAllMapTags(const annotated_map_msgs::TaggedPolygonalMap& map);



} //end namespace


#endif

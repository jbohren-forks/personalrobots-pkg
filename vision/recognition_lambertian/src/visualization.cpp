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

// Author: Marius Muja


#include <recognition_lambertian/visualization.h>
#include <visualization_msgs/Marker.h>

void publishNormals(ros::Node* node, robot_msgs::PointCloud points, vector<robot_msgs::Vector3> coeff, float length)
{
	node->advertise<visualization_msgs::Marker>("visualization_marker", 1);

	visualization_msgs::Marker marker;
	marker.header.frame_id = points.header.frame_id;
	marker.header.stamp = ros::Time((uint64_t)0ULL);
	marker.ns = "normals";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.color.a = 1.0;
	marker.color.g = 1.0;

	marker.set_points_size(2*points.get_pts_size());

	for (size_t i=0;i<points.get_pts_size();++i) {

		marker.points[2*i].x = points.pts[i].x;
		marker.points[2*i].y = points.pts[i].y;
		marker.points[2*i].z = points.pts[i].z;

		marker.points[2*i+1].x = points.pts[i].x+length*coeff[i].x;
		marker.points[2*i+1].y = points.pts[i].y+length*coeff[i].y;
		marker.points[2*i+1].z = points.pts[i].z+length*coeff[i].z;

	}
	node->publish( "visualization_marker", marker );

}

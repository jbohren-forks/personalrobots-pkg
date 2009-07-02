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

//! \author Alex Sorokin 

#include "ros/node.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"


#include <deque>
#include <vector>

#include "point_cloud_mapping/kdtree/kdtree_ann.h"
//include "point_cloud_mapping/kdtree/kdtree_flann.h"
#include "boost/thread.hpp"
#include "math.h"



#include "robot_msgs/PointCloud.h"

namespace pcd_misc
{

  /** \brief split point cloud into disjoint components
   *
   */
  void cluster_pcd_points(const robot_msgs::PointCloud& centers,double max_radius,std::vector<int>& cluster_ids, unsigned int& num_clusters);

//void pcd_split_into_clusters(const robot_msgs::PointCloud& source,const std::vector<int>& cluster_ids, const int& num_clusters, std::vector<robot_msgs::PointCloud>& clouds_out );

void cluster_ids_to_cluster_indices(const std::vector<int>& cluster_ids, const unsigned int& num_clusters, std::vector<std::vector<int> >& clouds_by_indices_out );


void variationAlongLine(robot_msgs::Point32 dir_line,robot_msgs::Point32 pt_line, robot_msgs::PointCloud cloud, std::vector<int> indices, float &min_v,float& max_v);
}

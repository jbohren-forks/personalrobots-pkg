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


#include "pcd_misc.h"


using namespace robot_msgs;
using namespace pcd_misc;

int union_get_parent(std::vector<int>& group_ids,int i)
{
    while(i != group_ids[i])
      i=group_ids[i];
    return i;
}
void union_compress_path(std::vector<int>& group_ids,int i,int parent)
{
    while(i != group_ids[i])
    {
      int j=group_ids[i];
      group_ids[i]=parent;
      i=j;
    }
    if(group_ids[i] != parent)
    {
      group_ids[i] = parent;
    }
}
void union_compress_path(std::vector<int>& group_ids,int i)
{
  int parent=union_get_parent(group_ids,i);
  union_compress_path(group_ids,i,parent);
}

void union_merge(std::vector<int>& group_ids,std::vector<int>& group_counts, int i,int j)
{
  int i_parent=union_get_parent(group_ids,i);
  int j_parent=union_get_parent(group_ids,j);
  int parent;
  if(group_counts[i_parent]>group_counts[j_parent])
  {
    parent=i_parent;
  }
  else
  {
    parent=j_parent;
  }
  group_counts[parent]=group_counts[i_parent]+group_counts[j_parent];

  union_compress_path(group_ids,i,parent);
  union_compress_path(group_ids,j,parent);

}



void pcd_misc::cluster_pcd_points(const sensor_msgs::PointCloud& centers,
                                  double max_radius,
                                  std::vector<int>& cluster_ids_final, 
                                  unsigned int& num_clusters)
{
  unsigned int num_pts=centers.pts.size();
  

  std::vector<int> cluster_ids;
  std::vector<int> cluster_sizes;

  cluster_ids.resize(num_pts);
  cluster_sizes.resize(num_pts);
  cluster_ids_final.resize(num_pts);
  
  ROS_DEBUG("Building KDtree");
  //Build kd-tree on centers of mass
  cloud_kdtree::KdTreeANN kd_tree(centers);

  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    cluster_ids[iPt]=iPt;
    cluster_sizes[iPt]=1;
  }
  
  //Link nearby polygons with kd-tree
  for(unsigned int iPt=0;iPt<num_pts;iPt++){
    std::vector<int> k_indices;
    std::vector<float> k_distances;

    kd_tree.radiusSearch(centers, iPt, max_radius, k_indices, k_distances);

    for(unsigned int iI=0;iI<k_indices.size();iI++)
    {
      if(k_distances[iI]<max_radius)
      {
        union_merge(cluster_ids,cluster_sizes,int(iPt),k_indices[iI]);
      }
    }
  }
  
  ROS_DEBUG("Union-find unification and path compression");
  //Union find: parent finding and path compression
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    union_compress_path(cluster_ids,iPt);
  }
  

  ROS_DEBUG("Merging the labels");
  //Here we rely that we'll see the root of the union first.
  //We simply copy the root.
  //For non-root elements, we merge them
  unsigned int iPtOut=0;
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    if(cluster_ids[iPt]==int(iPt))
    {
      cluster_ids_final[iPt]=iPtOut;
      
      iPtOut++;
    }
  }
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    if(cluster_ids[iPt]!=(int)iPt)
    {
      int clusterOutID=cluster_ids_final[cluster_ids[iPt]];
      cluster_ids_final[iPt]=clusterOutID;
    }
  }
  num_clusters=iPtOut;

  return;
}




void pcd_misc::cluster_ids_to_cluster_indices(const std::vector<int>& cluster_ids, const unsigned int& num_clusters, std::vector<std::vector<int> >& clouds_by_indices_out )
{
  unsigned int num_pts=cluster_ids.size();

  clouds_by_indices_out.resize(num_clusters);
  for(unsigned int iC=0;iC<num_clusters;iC++)
  {
    std::vector<int>& cluster_members=clouds_by_indices_out[iC];

    unsigned int num_in_cluster=0;
    for(unsigned int iPt=0;iPt<num_pts;iPt++)
    {
      if(cluster_ids[iPt]==(int)iC)
      {
        num_in_cluster++;
      }
    }

    cluster_members.resize(num_in_cluster);
    unsigned int out_location=0;
    for(unsigned int iPt=0;iPt<num_pts;iPt++)
    {
      if(cluster_ids[iPt]==(int)iC)
      {
        cluster_members[out_location]=iPt;
        out_location++;
      }
    }

  }
}



void pcd_misc::variationAlongLine(Point32 dir_line,geometry_msgs::Point32 pt_line, sensor_msgs::PointCloud cloud, std::vector<int> indices, float &min_v,float& max_v)
{
  min_v=1e15;
  max_v=-1e15;
  for(unsigned int iPt=0;iPt<indices.size();iPt++)
  {
    Point32 &pt=cloud.pts[indices[iPt]];
    float new_v=(pt.x-pt_line.x)*dir_line.x+(pt.y-pt_line.y)*dir_line.y+(pt.z-pt_line.z)*dir_line.z;

    if(min_v>new_v)
      min_v=new_v;
    if(max_v<new_v)
      max_v=new_v;
  }
}

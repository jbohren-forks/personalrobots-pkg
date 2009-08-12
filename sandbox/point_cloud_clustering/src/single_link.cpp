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

#include <point_cloud_clustering/single_link.h>

using namespace point_cloud_clustering;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SingleLink::SingleLink(double max_radius)
{
  max_radius_ = max_radius;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SingleLink::unionGetParent(std::vector<int>& group_ids, int i)
{
  while (i != group_ids[i])
    i = group_ids[i];
  return i;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SingleLink::unionCompressPath(std::vector<int>& group_ids, int i, int parent)
{
  while (i != group_ids[i])
  {
    int j = group_ids[i];
    group_ids[i] = parent;
    i = j;
  }
  if (group_ids[i] != parent)
  {
    group_ids[i] = parent;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SingleLink::unionCompressPath(std::vector<int>& group_ids, int i)
{
  int parent = unionGetParent(group_ids, i);
  unionCompressPath(group_ids, i, parent);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SingleLink::unionMerge(std::vector<int>& group_ids, std::vector<int>& group_counts, int i, int j)
{
  int i_parent = unionGetParent(group_ids, i);
  int j_parent = unionGetParent(group_ids, j);
  int parent;
  if (group_counts[i_parent] > group_counts[j_parent])
  {
    parent = i_parent;
  }
  else
  {
    parent = j_parent;
  }
  group_counts[parent] = group_counts[i_parent] + group_counts[j_parent];

  unionCompressPath(group_ids, i, parent);
  unionCompressPath(group_ids, j, parent);

}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SingleLink::singleLinkCluster(const sensor_msgs::PointCloud& centers,
                                   cloud_kdtree::KdTree& pt_cloud_kdtree,
                                   std::vector<int>& cluster_ids_final)
{
  unsigned int num_pts = centers.points.size();

  std::vector<int> cluster_ids;
  std::vector<int> cluster_sizes;

  cluster_ids.resize(num_pts);
  cluster_sizes.resize(num_pts);
  cluster_ids_final.resize(num_pts);

  for (unsigned int iPt = 0 ; iPt < num_pts ; iPt++)
  {
    cluster_ids[iPt] = iPt;
    cluster_sizes[iPt] = 1;
  }

  //Link nearby polygons with kd-tree
  for (unsigned int iPt = 0 ; iPt < num_pts ; iPt++)
  {
    std::vector<int> k_indices;
    std::vector<float> k_distances;

    pt_cloud_kdtree.radiusSearch(centers, iPt, max_radius_, k_indices, k_distances);

    for (unsigned int iI = 0 ; iI < k_indices.size() ; iI++)
    {
      if (k_distances[iI] < max_radius_)
      {
        unionMerge(cluster_ids, cluster_sizes, int(iPt), k_indices[iI]);
      }
    }
  }

  ROS_DEBUG("Union-find unification and path compression");
  //Union find: parent finding and path compression
  for (unsigned int iPt = 0 ; iPt < num_pts ; iPt++)
  {
    unionCompressPath(cluster_ids, iPt);
  }

  ROS_DEBUG("Merging the labels");
  //Here we rely that we'll see the root of the union first.
  //We simply copy the root.
  //For non-root elements, we merge them
  unsigned int iPtOut = 0;
  for (unsigned int iPt = 0 ; iPt < num_pts ; iPt++)
  {
    if (cluster_ids[iPt] == int(iPt))
    {
      cluster_ids_final[iPt] = iPtOut;

      iPtOut++;
    }
  }
  for (unsigned int iPt = 0 ; iPt < num_pts ; iPt++)
  {
    if (cluster_ids[iPt] != (int) iPt)
    {
      int clusterOutID = cluster_ids_final[cluster_ids[iPt]];
      cluster_ids_final[iPt] = clusterOutID;
    }
  }

  return;
}

// --------------------------------------------------------------
/* See function definition
 * TODO: have Alex refactor this so not duplicating data */
// --------------------------------------------------------------
int SingleLink::cluster(const sensor_msgs::PointCloud& pt_cloud,
                        cloud_kdtree::KdTree& pt_cloud_kdtree,
                        const std::set<unsigned int>& indices_to_cluster,
                        std::map<unsigned int, std::vector<int> >& created_clusters)
{
  created_clusters.clear();
  unsigned int nbr_pts_full_cloud = pt_cloud.points.size();
  if (max_radius_ < 1e-6)
  {
    ROS_ERROR("SingleLink::cluster given negative radius");
    return -1;
  }

  // -----------------------------------------
  // Create a subset point cloud containing only those points specified to cluster over
  sensor_msgs::PointCloud subset_pt_cloud;
  subset_pt_cloud.points.resize(indices_to_cluster.size());
  unsigned int idx = 0;
  for (std::set<unsigned int>::const_iterator iter_indices_to_cluster = indices_to_cluster.begin() ; iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++, idx++)
  {
    if (*iter_indices_to_cluster > nbr_pts_full_cloud)
    {
      ROS_ERROR ("SingleLink::cluster given out of bounds index to cluster");
      return -1;
    }
    subset_pt_cloud.points[idx].x = pt_cloud.points[*iter_indices_to_cluster].x;
    subset_pt_cloud.points[idx].y = pt_cloud.points[*iter_indices_to_cluster].y;
    subset_pt_cloud.points[idx].z = pt_cloud.points[*iter_indices_to_cluster].z;
  }
  cloud_kdtree::KdTreeANN subset_kdtree(subset_pt_cloud);

  // -----------------------------------------
  // Cluster
  // cluster_ids: element is the cluster label for each point
  std::vector<int> cluster_ids;
  singleLinkCluster(subset_pt_cloud, subset_kdtree, cluster_ids);

  // -----------------------------------------
  // Create cluster map
  // cluster_label --> [point indices]
  idx = 0;
  for (std::set<unsigned int>::const_iterator iter_indices_to_cluster = indices_to_cluster.begin() ; iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++, idx++)
  {
    unsigned int curr_cluster_label = cluster_ids[idx];
    if (created_clusters.count(curr_cluster_label) == 0)
    {
      created_clusters[curr_cluster_label] = std::vector<int> ();
    }
    created_clusters[curr_cluster_label].push_back(*iter_indices_to_cluster);
  }
  return 0;
}

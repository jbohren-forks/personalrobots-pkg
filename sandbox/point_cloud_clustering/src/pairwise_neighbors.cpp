/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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

#include <point_cloud_clustering/pairwise_neighbors.h>

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int point_cloud_clustering::PairwiseNeighbors::setParameters(double radius, unsigned int nbr_neighbors)
{
  if (radius < 0.0)
  {
    ROS_ERROR("Invalid radius accuracy");
    return -1;
  }

  if (nbr_neighbors == 0)
  {
    ROS_ERROR("Invalid kmeans nbr_neighbors");
    return -1;
  }

  radius_ = radius;
  nbr_neighbors_ = nbr_neighbors;

  parameters_defined_ = true;
  return 0;
}

// --------------------------------------------------------------
/* See function definition
 * Important: if you change indices_to_cluster from a set to anything else,
 *            must modify adjacency list construction below */
// --------------------------------------------------------------
int point_cloud_clustering::PairwiseNeighbors::cluster(const robot_msgs::PointCloud& pt_cloud,
                                                       cloud_kdtree::KdTree& pt_cloud_kdtree,
                                                       const set<unsigned int>& indices_to_cluster,
                                                       map<unsigned int, vector<int> >& created_clusters,
                                                       map<unsigned int, vector<float> >* cluster_centroids)
{
  if (parameters_defined_ == false)
  {
    return -1;
  }

  unsigned int nbr_pts = pt_cloud.pts.size();

  // i --> [point i's neighbors j]
  // (i < j)
  map<unsigned int, set<unsigned int> > adj_list;

  // Iterate over each index, find neighboring points, and randomly link to them
  set<unsigned int>::const_iterator iter_indices_to_cluster;
  for (iter_indices_to_cluster = indices_to_cluster.begin(); iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    // retrieve next index
    unsigned int curr_idx = *iter_indices_to_cluster;
    if (curr_idx >= nbr_pts)
    {
      ROS_ERROR("Invalid index to cluster");
      return -1;
    }

    // create adj list entry if necessary
    if (adj_list.count(curr_idx) == 0)
    {
      adj_list[curr_idx] = set<unsigned int> ();
    }

    // range search radius_
    vector<int> neighbor_indices;
    vector<float> neighbor_distances;
    pt_cloud_kdtree.radiusSearch(curr_idx, radius_, neighbor_indices, neighbor_distances);

    // generate random indices
    unsigned int nbr_radius_neighbors = neighbor_indices.size();
    vector<unsigned int> random_indices(nbr_radius_neighbors, 0);
    for (unsigned int i = 0 ; i < nbr_radius_neighbors ; i++)
    {
      random_indices[i] = i;
    }
    random_shuffle(random_indices.begin(), random_indices.end());

    // Randomly link to neighbors within radius
    unsigned int nbr_linked_neighbors = 0;
    for (unsigned int i = 0 ; nbr_linked_neighbors < nbr_neighbors_ && i < nbr_radius_neighbors ; i++)
    {
      // retrieve neighboring index, verify it is valid
      unsigned int curr_neighbor_idx = random_indices[i];
      if (indices_to_cluster.count(curr_neighbor_idx) == 0)
      {
        continue;
      }

      // avoid self-edges
      if (curr_neighbor_idx == curr_idx)
      {
        continue;
      }

      // define edge (a,b) st. a < b
      unsigned int a = std::min(curr_idx, curr_neighbor_idx);
      unsigned int b = std::max(curr_idx, curr_neighbor_idx);
      adj_list[a].insert(b); // a guaranteed to exist since iterating in ascending order with set
      nbr_linked_neighbors++;
    }
  }

  // Create clusters from adjacency list
  created_clusters.clear();
  unsigned int cluster_id = 0;
  map<unsigned int, set<unsigned int> >::iterator iter_adj_list;
  for (iter_adj_list = adj_list.begin(); iter_adj_list != adj_list.end() ; iter_adj_list++)
  {
    unsigned int curr_source = iter_adj_list->first;
    set<unsigned int>& curr_neighbors = iter_adj_list->second;
    set<unsigned int>::iterator iter_neighbors;
    for (iter_neighbors = curr_neighbors.begin(); iter_neighbors != curr_neighbors.end() ; iter_neighbors++)
    {
      unsigned int curr_target = *iter_neighbors;
      created_clusters[cluster_id] = vector<int> (2);

      // Create edges based on z-coordinate
      if (pt_cloud.pts[curr_source].z < pt_cloud.pts[curr_target].z)
      {
        created_clusters[cluster_id][0] = static_cast<int> (curr_source);
        created_clusters[cluster_id][1] = static_cast<int> (curr_target);
      }
      else
      {
        created_clusters[cluster_id][0] = static_cast<int> (curr_target);
        created_clusters[cluster_id][1] = static_cast<int> (curr_source);
      }
      cluster_id++;

    }
  }

  return 0;
}

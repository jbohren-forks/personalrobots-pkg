#ifndef __PCC_PAIRWISE_NEIGHBORS_H__
#define __PCC_PAIRWISE_NEIGHBORS_H__
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

#include <stdlib.h>

#include <set>
#include <map>
#include <vector>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

#include <point_cloud_clustering/point_cloud_clustering.h>

// --------------------------------------------------------------
/*!
 * \file pairwise_neighbors.h
 *
 * \brief PairwiseNeighbors creates clusters of size 2 between adjacent
 *        points
 */
// --------------------------------------------------------------

namespace point_cloud_clustering
{
  // --------------------------------------------------------------
  /*!
   * \brief PairwiseNeighbors creates clusters of size 2 between adjacent
   *        points.
   *
   * This type of clustering is useful for creating edges in a graph
   * where the points are the nodes, which can be used for MRF-based
   * classification.
   */
  // --------------------------------------------------------------
  class PairwiseNeighbors: public PointCloudClustering
  {
    public:
      // --------------------------------------------------------------
      /*!
       * \brief Instantiates the clustering to create edges (clusters)
       *        between neighboring points.
       *
       * The neighborhood of a point is defined by a Euclidean distance.
       * The number of neighbors a point links to is specified as a parameter.
       * The order in which points are linked to neighbors is arbitrary.
       * The created edges are unique.  The edges are created such that the point
       * with lower z-coordinate has the first index.
       *
       * \param neighbor_radius The Euclidean radius that defines the neighborhood
       *                        for a point
       * \param max_nbr_neighbors The maximum number of neighbors a point should
       *                          link to.  0 indicates to link to all neighbors.
       *                          If the value is smaller than the neighborhood size,
       *                          the neighbors are chosen randomly.
       */
      // --------------------------------------------------------------
      PairwiseNeighbors(double neighbor_radius, unsigned int max_nbr_neighbors);

      // --------------------------------------------------------------
      /*!
       * \see PointCloudClustering::cluster
       */
      // --------------------------------------------------------------
      virtual int cluster(const sensor_msgs::PointCloud& pt_cloud,
                          cloud_kdtree::KdTree& pt_cloud_kdtree,
                          const std::set<unsigned int>& indices_to_cluster,
                          std::map<unsigned int, std::vector<int> >& created_clusters);

    private:
      double neighbor_radius_;
      unsigned int max_nbr_neighbors_;
  };
}
#endif

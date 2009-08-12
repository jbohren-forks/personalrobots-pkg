#ifndef __PCC_KMEANS_H__
#define __PCC_KMEANS_H__
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

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <point_cloud_clustering/point_cloud_clustering.h>

// --------------------------------------------------------------
/*!
 * \file single_link.h
 *
 * \brief Creates clusters in 3-D using single-link clustering based
 *        on proximity
 */
// --------------------------------------------------------------

namespace point_cloud_clustering
{
  // --------------------------------------------------------------
  /*!
   * \brief SingleLink implements single-link clustering for creating
   *        groups of points based on physical proximity.
   */
  // --------------------------------------------------------------
  class SingleLink: public PointCloudClustering
  {
    public:
      // --------------------------------------------------------------
      /*!
       * \brief Creates the clustering algorithm so all neighboring points
       *        with max_radius Euclidean distance are grouped together
       *
       * \param max_radius The radius each point uses to search for neighbors
       */
      // --------------------------------------------------------------
      SingleLink(double max_radius);

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
      int unionGetParent(std::vector<int>& group_ids, int i);

      void unionCompressPath(std::vector<int>& group_ids, int i, int parent);

      void unionCompressPath(std::vector<int>& group_ids, int i);

      void unionMerge(std::vector<int>& group_ids, std::vector<int>& group_counts, int i, int j);

      void singleLinkCluster(const sensor_msgs::PointCloud& centers,
                             cloud_kdtree::KdTree& pt_cloud_kdtree,
                             std::vector<int>& cluster_ids);

      double max_radius_;
  };
}
#endif

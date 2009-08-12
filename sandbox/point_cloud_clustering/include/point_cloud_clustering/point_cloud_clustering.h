#ifndef __PCC_H__
#define __PCC_H__
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

#include <set>
#include <map>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

// --------------------------------------------------------------
/*!
 * \file point_cloud_clustering.h
 *
 * \brief The abstract base class for all clustering methods that
 *        operate on sensor_msgs::PointCloud
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief The namespace of clustering methods that
 *        operate on sensor_msgs::PointCloud
 */
// --------------------------------------------------------------
namespace point_cloud_clustering
{
  // --------------------------------------------------------------
  /*!
   * \brief PointCloudClustering is the abstract base class for
   *        clustering methods that operate on 3-D data.
   *
   * All inheriting classes define all necessary parameters in their
   * constructor.  Hence, after instantiation, clustering can be done
   * through the cluster() method.  Use setStartingClusterLabel() to
   * ensure the created clusters' labels begin at a certain value (by
   * default it is 0).
   */
  // --------------------------------------------------------------
  class PointCloudClustering
  {
    public:
      // --------------------------------------------------------------
      /*!
       * \brief Computes the centroids of the given clusters
       *
       * \param pt_cloud The point cloud the clusters were created from
       * \param clusters The result from a previous PointCloudClustering::cluster call
       * \param cluster_centroids The computed cluster centroids where the key is
       *                          the original cluster label and the value is a
       *                          vector of size 3 with the x,y,z centroid
       *
       */
      // --------------------------------------------------------------
      static void
      computeClusterCentroids(const sensor_msgs::PointCloud& pt_cloud,
                              const std::map<unsigned int, std::vector<int> >& clusters,
                              std::map<unsigned int, std::vector<float> >& cluster_centroids);

      // --------------------------------------------------------------
      /*!
       * \brief Sets the cluster labels to start from 0
       */
      // --------------------------------------------------------------
      PointCloudClustering();

      virtual ~PointCloudClustering() = 0;

      // --------------------------------------------------------------
      /*!
       * \brief Sets the labels of the to-be created clusters to start from
       *        the specified value
       *
       * \param starting_label The starting label of the created clusters
       */
      // --------------------------------------------------------------
      inline void setStartingClusterLabel(unsigned int starting_label)
      {
        starting_label_ = starting_label;
      }

      // --------------------------------------------------------------
      /*!
       * \brief Clusters the entire point cloud
       *
       * \param pt_cloud The point cloud to cluster
       * \param pt_cloud_kdtree The data structure for efficient neighborhood searches
       * \param created_clusters The created clusters where the key is the cluster label
       *                         and the value are the indices in pt_cloud that belong
       *                         to the cluster
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      int cluster(const sensor_msgs::PointCloud& pt_cloud,
                  cloud_kdtree::KdTree& pt_cloud_kdtree,
                  std::map<unsigned int, std::vector<int> >& created_clusters);

      // --------------------------------------------------------------
      /*!
       * \brief Clusters only the specified indices of the point cloud
       *
       * \param pt_cloud The point cloud to cluster
       * \param pt_cloud_kdtree The data structure for efficient neighborhood searches
       * \param indices_to_cluster The indices in pt_cloud to only cluster over
       * \param created_clusters The created clusters where the key is the cluster label
       *                         and the value are the indices in pt_cloud that belong
       *                         to the cluster
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      virtual int cluster(const sensor_msgs::PointCloud& pt_cloud,
                          cloud_kdtree::KdTree& pt_cloud_kdtree,
                          const std::set<unsigned int>& indices_to_cluster,
                          std::map<unsigned int, std::vector<int> >& created_clusters) = 0;

    protected:
      // --------------------------------------------------------------
      /*!
       * \brief Finds only the neighbors within the radius that were indicated to
       *        be clustered over
       *
       * \param pt_cloud_kdtree The data structure for efficient neighborhood searches
       * \param index The index in the original point cloud to find a neighborhood for
       * \param radius The Euclidean radius that defines the neighborhood
       * \param indices_to_cluster The indices in pt_cloud to only cluster over
       * \param neighbor_indices The resulting indices in the point cloud that are
       *                         neighbors of index
       */
      // --------------------------------------------------------------
      void findRadiusNeighbors(cloud_kdtree::KdTree& pt_cloud_kdtree,
                               unsigned int index,
                               double radius,
                               const std::set<unsigned int>& indices_to_cluster,
                               std::vector<unsigned int>& neighbor_indices);

      unsigned int starting_label_;
  };
}

#endif

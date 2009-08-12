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

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

#include <point_cloud_clustering/point_cloud_clustering.h>

// --------------------------------------------------------------
/*!
 * \file kmeans.h
 *
 * \brief KMeans performs basic k-means clustering on the point
 *        cloud
 */
// --------------------------------------------------------------

namespace point_cloud_clustering
{
  // --------------------------------------------------------------
  /*!
   * \brief A KMeans clustering performs k-means clustering on the point
   *        cloud where the features are the x,y,z coordinates
   */
  // --------------------------------------------------------------
  class KMeans: public PointCloudClustering
  {
    public:
      // --------------------------------------------------------------
      /*!
       * \brief Instantiates the k-means clustering with the specified
       *        parameters
       *
       * This method wraps around OpenCV's k-means implementation:
       * http://opencv.willowgarage.com/documentation/miscellaneous_functions.html?highlight=kmeans#cvKMeans2
       *
       * \param k_factor Defines the number of clusters to produce where
       *                 k = k_factor * [number of points to cluster over]
       * \param accuracy The termination accuracy criterea, see OpenCV
       *                 documentation
       * \param max_iter The maximum number of iterations before terminating,
       *                 see OpenCV documentation
       */
      // --------------------------------------------------------------
      KMeans(double k_factor, double accuracy, unsigned int max_iter);

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
      double k_factor_;
      double accuracy_;
      unsigned int max_iter_;
  };
}
#endif

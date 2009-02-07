/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_KDTREE_H_
#define _CLOUD_KDTREE_KDTREE_H_

// ROS includes
#include <std_msgs/PointCloud.h>
#include <std_msgs/Point32.h>

#include <stdlib.h>

#define USE_ANN

#ifdef USE_ANN
  #include <ANN/ANN.h>
#else
  #include <flann.h>
  typedef int      ANNidx;            // define ANNidx for FL-ANN
  typedef float    ANNdist;           // define ANNdist for FL-ANN as float instead of double
  typedef ANNidx*  ANNidxArray;
  typedef ANNdist* ANNdistArray;
  typedef float*   ANNpoint;
  typedef float*   ANNpointArray;     // float* instead of float** for FL-ANN
#endif

namespace cloud_kdtree
{
  class KdTree
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults.
        */
      KdTree ()
      {
        bucket_size_ = 30;    // default bucket size value
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)
        nn_idx_      = NULL;
        nn_dists_    = NULL;
        points_      = NULL;
#ifdef USE_ANN
        ann_kd_tree_ = NULL;
#else
        index_id_    = NULL;
        flann_log_verbosity (LOG_NONE);
#endif
        nr_points_   = 0;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTree (std_msgs::PointCloud *points)
      {
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)
        bucket_size_ = 30;    // default bucket size value

        // Allocate enough data
        nr_points_ = convertCloudToArray (points, points_);
        nn_idx_    = new ANNidx [nr_points_];
        nn_dists_  = new ANNdist [nr_points_];
        // Create the kd_tree representation
#ifdef USE_ANN
        ann_kd_tree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
#else
        float speedup;
        flann_param_.algorithm = KDTREE;
        flann_param_.log_level = LOG_NONE;
        flann_param_.log_destination = NULL;

        flann_param_.checks = 32;
        flann_param_.trees = 8;
        flann_param_.branching = 32;
        flann_param_.iterations = 7;
        flann_param_.target_precision = -1;
        index_id_    = flann_build_index (points_, nr_points_, dim_, &speedup, &flann_param_);
        flann_log_verbosity (LOG_NONE);
#endif
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \note ATTENTION: This method breaks the 1-1 mapping between the indices returned using \a getNeighborsIndices
        * and the ones from the \a points message ! When using this method, make sure to get the underlying point data
        * using the \a getPoint method
        * \param points the ROS point cloud data array
        * \param indices the point cloud indices
        */
      KdTree (std_msgs::PointCloud *points, std::vector<int> *indices)
      {
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)
        bucket_size_ = 30;    // default bucket size value

        // Allocate enough data
        nr_points_ = convertCloudToArray (points, indices, points_);
        nn_idx_    = new ANNidx [nr_points_];
        nn_dists_  = new ANNdist [nr_points_];
        // Create the kd_tree representation
#ifdef USE_ANN
        ann_kd_tree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
#else
        float speedup;
        index_id_    = flann_build_index (points_, nr_points_, dim_, &speedup, &flann_param_);
        flann_log_verbosity (LOG_NONE);
#endif
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        * \param dim specify the number of extra channels that we want to include as part of a point. For example if
        * the first 3 channels in \a points are \<r,g,b\>, and dim equals 3, a resulting data point will contain
        * a 6D tuple: \<x,y,z,r,g,b\> and all subsequent nearest neighbor searches will be performed in that space.
        */
      KdTree (std_msgs::PointCloud *points, int dim)
      {
        dim_       = 3 + dim;     // default number of dimensions (3 = xyz) + the extra channels

        // Allocate enough data
        nr_points_ = convertCloudToArray (points, dim, points_);
        nn_idx_   = new ANNidx [nr_points_];
        nn_dists_ = new ANNdist [nr_points_];
        // Create the kd_tree representation
#ifdef USE_ANN
        ann_kd_tree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
#else
        float speedup;
        index_id_    = flann_build_index (points_, nr_points_, dim_, &speedup, &flann_param_);
        flann_log_verbosity (LOG_NONE);
#endif
      }


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTree ()
      {
        // Data array cleanup
        if (points_ != NULL && nr_points_ != 0)
          annDeallocPts (points_);

        // ANN Cleanup
        if (nn_idx_ != NULL)      delete [] nn_idx_;
        if (nn_dists_ != NULL)    delete [] nn_dists_;
#ifdef USE_ANN
        if (ann_kd_tree_ != NULL) delete ann_kd_tree_;
        ann_kd_tree_ = NULL;
        annClose ();
#else
        flann_free_index (index_id_, &flann_param_);
#endif
      }

      int convertCloudToArray (std_msgs::PointCloud *ros_cloud, ANNpointArray &ann_cloud);
      int convertCloudToArray (std_msgs::PointCloud *ros_cloud, std::vector<int> *indices, ANNpointArray &ann_cloud);
      int convertCloudToArray (std_msgs::PointCloud *ros_cloud, unsigned int nr_dimensions, ANNpointArray &ann_cloud);
      int convertCloudToArray (std_msgs::PointCloud *ros_cloud, std::vector<unsigned int> dimensions, ANNpointArray &ann_cloud);

      bool nearestKSearch (std_msgs::Point32 *p_q, int k);
      bool nearestKSearch (std_msgs::PointCloud *points, unsigned int index, int k);
      bool nearestKSearch (int p_idx, int k);
      bool nearestKSearch (int p_idx, int k, std::vector<int> &indices, std::vector<double> &distances);

      bool radiusSearch (std_msgs::Point32 *p_q, double radius, int max_nn = INT_MAX);
      bool radiusSearch (std_msgs::PointCloud *points, unsigned int index, double radius, int max_nn = INT_MAX);

      void radiusSearch (unsigned int index, double radius, int max_nn = INT_MAX);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Obtain the neighbors' point indices on the last nearestKSearch or radiusSearch
        * \param indices vector container for neighbor point indices storage
        */
      inline void
        getNeighborsIndices (std::vector<int> &indices)
      {
        int nr_neighbors;
        if (last_call_type_  == K_SEARCH)
          nr_neighbors = k_;
        else
          nr_neighbors = neighbors_in_radius_;

        indices.resize (nr_neighbors);
        for (int i = 0; i < nr_neighbors; i++)
          indices[i] = nn_idx_[i];
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Obtain all the distances from the query point to its neighbors from the last nearestKSearch or
        * radiusSearch calls.
        * \param distances vector container for distances from the query point to all its neighbors
        */
      inline void
        getNeighborsDistances (std::vector<double> &distances)
      {
        int nr_neighbors;
        if (last_call_type_ == K_SEARCH)
          nr_neighbors = k_;
        else
          nr_neighbors = neighbors_in_radius_;

        distances.resize (nr_neighbors);
        for (int i = 0; i < nr_neighbors; i++)
          distances[i] = nn_dists_[i];
      }


#ifndef USE_ANN
      // Define bogus ANN wrappers for FL-ANN
      inline ANNpoint annAllocPt    (int dim)           { ANNpoint p = (float*)malloc (dim * sizeof (float)); return (p); }
      inline ANNpoint annAllocPts   (int n, int dim)    { ANNpointArray p = (float*)malloc (n * dim * sizeof (float)); return (p); }
      inline void     annDeallocPt  (ANNpoint &p)       { free (p); p = NULL; }
      inline void     annDeallocPts (ANNpointArray &pa) { free (pa); pa = NULL; }
#endif

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the value of a point located at the given index as a Point32 message
        * \note This method returns the data at the underlying point index with respect to the internal cloud model!
        * The mapping is usually 1-1 with the PointCloud message given in the KdTree constructor, unless pointed otherwise.
        * \param index the point index
        * \param point the point data
        */
      inline void
        getPoint (int index, std_msgs::Point32 &point)
      {
        point.x = points_[index][0];
        point.y = points_[index][1];
        point.z = points_[index][2];
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the underlying 3D x,y,z values of a point given its index in the cloud
        * \param idx the point index
        * \param x the resultant X coordinate value of the point
        * \param y the resultant Y coordinate value of the point
        * \param z the resultant Z coordinate value of the point
        */
      inline void
        get3DPoint (int idx, float &x, float &y, float &z)
      {
        x = points_[idx][0];
        y = points_[idx][1];
        z = points_[idx][2];
      }

    private:
#ifdef USE_ANN
      /** \brief The ANN kd tree object */
      ANNkd_tree* ann_kd_tree_;
#else
      /** \brief A FL-ANN type index reference */
      FLANN_INDEX index_id_;
      /** \brief A pointer to a FL-ANN parameter structure */
      FLANNParameters flann_param_;
#endif

      /** \brief Nearest neighbors indices in ANN format */
      ANNidxArray nn_idx_;

      /** \brief Nearest neighbors distances in ANN format */
      ANNdistArray nn_dists_;

      /** \brief The requested number of nearest neighbors to search for in a \a nearestKSearch () call */
      int k_;

      /** \brief The number of nearest neighbors found in the last radius search */
      int neighbors_in_radius_;

      /** \brief Epsilon precision (error bound) for nearest neighbors searches */
      double epsilon_;

      /** \brief Internal tree bucket size */
      double bucket_size_;

      /** \brief Internal pointer to data */
      ANNpointArray points_;

      /** \brief Number of points in the tree */
      int nr_points_;
      /** \brief Tree dimensionality (i.e. the number of dimensions per point) */
      int dim_;

      /** \brief The library handles two different types of searches: K-nearest neighbor, and Radius search */
      enum searchStates { K_SEARCH, RADIUS_SEARCH };

      /** \brief The type of the last call (see \a searchStates) */
      int last_call_type_;
  };

}

#endif

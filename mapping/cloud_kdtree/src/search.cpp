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

#include "cloud_kdtree/kdtree.h"

namespace cloud_kdtree
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for k-nearest neighbors for the given query point.
    * \param p_q the given query point
    * \param k the number of neighbors to search for
    */
  bool
    KdTree::nearestKSearch (std_msgs::Point32 p_q, int k)
  {
    if (dim_ != 3)          // We want to discourage 3-D searching when the tree is creating for a different n-D
      return (false);

    ANNpoint p = annAllocPt (3);
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;
#ifdef USE_ANN
    ann_kd_tree_->annkSearch (p, k, nn_idx_, nn_dists_, epsilon_);
#else
    flann_find_nearest_neighbors_index (index_id_, p, 1, nn_idx_, nn_dists_, k, flann_param_.checks, &flann_param_);
#endif
    annDeallocPt (p);

    k_ = k;
    last_call_type_ = K_SEARCH;
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for k-nearest neighbors for a given query point, specified by a PointCloud message and an index.
    * \param points the point cloud data
    * \param index the index in \a points representing the query point
    * \param k the number of neighbors to search for
    */
  bool
    KdTree::nearestKSearch (std_msgs::PointCloud *points, unsigned int index, int k)
  {
    if (dim_ > (3 + (int)points->chan.size ()))  // Presume that the user know what he's doing, but check for overflow
      return (false);
    if (index >= points->pts.size ())
      return (false);

    ANNpoint p = annAllocPt (dim_);
    p[0] = points->pts.at (index).x; p[1] = points->pts.at (index).y; p[2] = points->pts.at (index).z;
    for (int d = 0; d < dim_ - 3; d++)
      p[d + 3] = points->chan[d].vals.at (index);

#ifdef USE_ANN
    ann_kd_tree_->annkSearch (p, k, nn_idx_, nn_dists_, epsilon_);
#else
    flann_find_nearest_neighbors_index (index_id_, p, 1, nn_idx_, nn_dists_, k, flann_param_.checks, &flann_param_);
#endif
    annDeallocPt (p);

    k_ = k;
    last_call_type_ = K_SEARCH;
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for k-nearest neighbors for the given query point index.
    * \note This method is to be used for extremely fast operations where we want to skip converting the point to
    * a \a Point32 message in a loop. There are no internal checks to validate the given index so use it carefully!
    * \param p_idx the given query point index
    * \param k the number of neighbors to search for
    */
  bool
    KdTree::nearestKSearch (int p_idx, int k)
  {
#ifdef USE_ANN
    ann_kd_tree_->annkSearch (points_[p_idx], k, nn_idx_, nn_dists_, epsilon_);
//     std::cerr << points_[p_idx][0] << " " << points_[p_idx][1] << " " << points_[p_idx][2] << std::endl;
#else
    ANNpoint p = annAllocPt (3);
    p[0] = points_[p_idx * dim_ * sizeof (float) + 0];
    p[1] = points_[p_idx * dim_ * sizeof (float) + 1];
    p[2] = points_[p_idx * dim_ * sizeof (float) + 2];
//     std::cerr << p[0] << " " << p[1] << " " << p[2] << std::endl;
    flann_find_nearest_neighbors_index (index_id_, p, 1, nn_idx_, nn_dists_, k, flann_param_.checks, &flann_param_);
    annDeallocPt (p);
#endif

    k_ = k;
    last_call_type_ = K_SEARCH;
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for all the nearest neighbors of the query point in a given radius.
    * \param p_q the given query point
    * \param radius the radius of the sphere bounding all of \a p_q 's neighbors
    * \param max_nn if given, bounds the maximum returned neighbors to this value
    */
  bool
    KdTree::radiusSearch (std_msgs::Point32 p_q, double radius, int max_nn)
  {
    if (dim_ != 3)          // We want to discourage 3-D searching when the tree is creating for a different n-D
      return (false);

    ANNpoint p = annAllocPt (3);
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;

#ifdef USE_ANN
    neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius * radius, 0, nn_idx_, nn_dists_, epsilon_);
    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_  = max_nn;
    ann_kd_tree_->annkFRSearch (p, radius * radius, neighbors_in_radius_, nn_idx_, nn_dists_, epsilon_);
#else
    flann_radius_search (index_id_, p, nn_idx_, nn_dists_, nr_points_, radius, flann_param_.checks, &flann_param_);
//LIBSPEC int flann_radius_search(FLANN_INDEX index_ptr, /* the index */
//                                        float* query,    /* query point */
//                                        int* indices, /* array for storing the indices found (will be modified) */
//                                        float* dists, /* similar but for storing distances */
//                                        int max_nn,  /* size of arrays indices and dists */
//                                        float radius, /* search radius (squared radius for euclidian metric) */
//                                        int checks,  /* number of features to check, sets the level of approximation */
//                                        FLANNParameters* flann_params);

#endif
    annDeallocPt (p);

    last_call_type_ = RADIUS_SEARCH;
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for all the nearest neighbors of a query point, specified by a PointCloud message and an index,
    * in a given radius.
    * \param points the point cloud data
    * \param index the index in \a points representing the query point
    * \param radius the radius of the sphere bounding all the query point's neighbors
    * \param max_nn if given, bounds the maximum returned neighbors to this value
    */
  bool
    KdTree::radiusSearch (std_msgs::PointCloud *points, unsigned int index, double radius, int max_nn)
  {
    if (dim_ > (3 + (int)points->chan.size ()))  // Presume that the user know what he's doing, but check for overflow
      return (false);
    if (index >= points->pts.size ())
      return (false);

    ANNpoint p = annAllocPt (dim_);
    p[0] = points->pts.at (index).x; p[1] = points->pts.at (index).y; p[2] = points->pts.at (index).z;
    for (int d = 0; d < dim_ - 3; d++)
      p[d + 3] = points->chan[d].vals.at (index);

#ifdef USE_ANN
    neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius * radius, 0, nn_idx_, nn_dists_, epsilon_);
    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_  = max_nn;
    ann_kd_tree_->annkFRSearch (p, radius * radius, neighbors_in_radius_, nn_idx_, nn_dists_, epsilon_);
#else
    flann_radius_search (index_id_, p, nn_idx_, nn_dists_, nr_points_, radius, flann_param_.checks, &flann_param_);
#endif
    annDeallocPt (p);

    last_call_type_ = RADIUS_SEARCH;
    return (true);
  }
}

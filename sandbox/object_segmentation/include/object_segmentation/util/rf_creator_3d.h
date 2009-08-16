#ifndef __RF_CREATOR_3D_H__
#define __RF_CREATOR_3D_H__
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

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <point_cloud_clustering/point_cloud_clustering.h>

#include <descriptors_3d/descriptor_3d.h>

#include <functional_m3n/random_field.h>

class RFCreator3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates creator to make RandomFields with the given
     *        feature and clustering parameters
     *
     * \param node_feature_descriptors
     * \param clique_set_feature_descriptors
     * \param clique_set_clusterings clique_set->[(cluster_only_nodes, clustering]
     */
    // --------------------------------------------------------------
    RFCreator3D(const std::vector<Descriptor3D*>& node_feature_descriptors,
                const std::vector<std::vector<Descriptor3D*> >& clique_set_feature_descriptors,
                const std::vector<std::vector<std::pair<bool,
                    point_cloud_clustering::PointCloudClustering*> > >& clique_set_clusterings);

    // only be done on clique-sets that contain edges (cliques of size 2)
    void setCSConcatenateNodeFeatures(const std::set<unsigned int> cs_indices);

    boost::shared_ptr<RandomField> createRandomField(const sensor_msgs::PointCloud& pt_cloud);

    // use_only_labeled = only create nodes for points with labels[i] != UNKNOWN_LABEL
    boost::shared_ptr<RandomField> createRandomField(const sensor_msgs::PointCloud& pt_cloud,
                                                     const std::vector<unsigned int>& labels,
                                                     const bool use_only_labeled);

  private:
    void createNodes(RandomField& rf,
                     const sensor_msgs::PointCloud& pt_cloud,
                     cloud_kdtree::KdTree& pt_cloud_kdtree,
                     const std::vector<unsigned int>& labels,
                     const bool use_only_labeled,
                     std::set<unsigned int>& node_indices);

    void createCliqueSet(RandomField& rf,
                         const sensor_msgs::PointCloud& pt_cloud,
                         cloud_kdtree::KdTree& pt_cloud_kdtree,
                         const std::set<unsigned int>& node_indices,
                         const unsigned int clique_set_idx);

    unsigned int
    concatenateNodeFeatures(const RandomField& rf,
                            const std::map<unsigned int, std::vector<int> >& created_clusters,
                            std::vector<boost::shared_array<const float> >& concatenated_features);

    unsigned int nbr_clique_sets_;
    std::vector<Descriptor3D*> node_feature_descriptors_;
    std::vector<std::vector<Descriptor3D*> > clique_set_feature_descriptors_;
    std::vector<std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > >
        clique_set_clusterings_;

    std::set<unsigned int> concat_nodes_cs_indices_;
};

#endif

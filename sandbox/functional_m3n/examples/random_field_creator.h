#ifndef __RF_CREATOR_H__
#define __RF_CREATOR_H__
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

#include <robot_msgs/PointCloud.h>

#include <point_cloud_clustering/kmeans.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <descriptors_3d/all_descriptors.h>

#include <functional_m3n/random_field.h>

class RandomFieldCreator
{
  public:
    RandomFieldCreator()
    {
    }

    const RandomField* createRandomField(const robot_msgs::PointCloud pt_cloud)
    {
      std::vector<float> labels;
      return createRandomField(pt_cloud, labels);
    }

    const RandomField* createRandomField(const robot_msgs::PointCloud pt_cloud,
                                         const std::vector<float>& labels);
  private:
    void createDescriptors();

    void createNodes(RandomField& rf,
                     const robot_msgs::PointCloud& pt_cloud,
                     cloud_kdtree::KdTree& pt_cloud_kdtree,
                     const vector<float>& labels,
                     set<unsigned int>& failed_indices);

    void createCliqueSet(RandomField& rf,
                         const robot_msgs::PointCloud& pt_cloud,
                         cloud_kdtree::KdTree& pt_cloud_kdtree,
                         const set<unsigned int>& node_indices,
                         const unsigned int clique_set_idx);

    std::vector<Descriptor3D*> node_feature_descriptors_;
    std::vector<std::vector<Descriptor3D*> > clique_set_feature_descriptors_;
    // use only nodes for clustering
    std::vector<std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > >
        clique_set_clusterings_;
};

#endif

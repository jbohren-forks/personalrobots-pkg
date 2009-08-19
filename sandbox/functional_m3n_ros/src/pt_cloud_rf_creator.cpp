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

#include <functional_m3n_ros/pt_cloud_rf_creator.h>

using namespace std;

PtCloudRFCreator::PtCloudRFCreator()
{
  vector<Descriptor3D*> node_feature_descriptors;
  vector<vector<Descriptor3D*> > clique_set_feature_descriptors;
  vector<vector<pair<bool, point_cloud_clustering::PointCloudClustering*> > >
      clique_set_clusterings;

  // ---------------------------------------------------------
  // Node features
  SpectralAnalysis* sa_nodes = new SpectralAnalysis(0.15);
  ShapeSpectral* ss_nodes = new ShapeSpectral(*sa_nodes);
  OrientationTangent* ot_nodes = new OrientationTangent(0, 0, 1.0, *sa_nodes);
  OrientationNormal* on_nodes = new OrientationNormal(0, 0, 1.0, *sa_nodes);
  Position* p_nodes = new Position();
  //
  node_feature_descriptors.push_back(ss_nodes);
  node_feature_descriptors.push_back(ot_nodes);
  node_feature_descriptors.push_back(on_nodes);
  node_feature_descriptors.push_back(p_nodes);

  // ---------------------------------------------------------
  // Clique set 0 features
  SpectralAnalysis* sa_cs0 = new SpectralAnalysis(0.2286);
  ShapeSpectral* ss_cs0 = new ShapeSpectral(*sa_cs0);
  OrientationTangent* ot_cs0 = new OrientationTangent(0, 0, 1.0, *sa_cs0);
  OrientationNormal* on_cs0 = new OrientationNormal(0, 0, 1.0, *sa_cs0);
  Position* p_cs0 = new Position();
  SpinImageCustom* sic_cs0 = new SpinImageCustom(0, 0, 1.0, 0.0762, 0.0762, 5, 4, false);
  BoundingBoxSpectral* bbs_cs0 = new BoundingBoxSpectral(-1.0, *sa_cs0);
  //
  vector<Descriptor3D*> cs0_feature_descriptors;
  cs0_feature_descriptors.push_back(ss_cs0);
  cs0_feature_descriptors.push_back(ot_cs0);
  cs0_feature_descriptors.push_back(on_cs0);
  cs0_feature_descriptors.push_back(p_cs0);
  cs0_feature_descriptors.push_back(sic_cs0);
  cs0_feature_descriptors.push_back(bbs_cs0);
  clique_set_feature_descriptors.push_back(cs0_feature_descriptors);

  // ---------------------------------------------------------
  // Clique set 0 clusterings
  point_cloud_clustering::KMeans* kmeans_cs0 = new point_cloud_clustering::KMeans(0.003, 2);
  std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > cs0_clusterings(1);
  cs0_clusterings[0].first = false; // false means cluster over ALL points (not just nodes)
  cs0_clusterings[0].second = kmeans_cs0;
  clique_set_clusterings.push_back(cs0_clusterings);

  // ---------------------------------------------------------
  // Clique set 1 features
  SpectralAnalysis* sa_cs1 = new SpectralAnalysis(-1);
  ShapeSpectral* ss_cs1 = new ShapeSpectral(*sa_cs1);
  OrientationTangent* ot_cs1 = new OrientationTangent(0, 0, 1.0, *sa_cs1);
  OrientationNormal* on_cs1 = new OrientationNormal(0, 0, 1.0, *sa_cs1);
  Position* p_cs1 = new Position();
  SpinImageNormal* sin_cs1 = new SpinImageNormal(0.0762, 0.0762, 7, 5, false, *sa_cs1);
  BoundingBoxSpectral* bbs_cs1 = new BoundingBoxSpectral(-1.0, *sa_cs1);
  //
  vector<Descriptor3D*> cs1_feature_descriptors;
  cs1_feature_descriptors.push_back(ss_cs1);
  cs1_feature_descriptors.push_back(ot_cs1);
  cs1_feature_descriptors.push_back(on_cs1);
  cs1_feature_descriptors.push_back(p_cs1);
  cs1_feature_descriptors.push_back(sin_cs1);
  cs1_feature_descriptors.push_back(bbs_cs1);
  clique_set_feature_descriptors.push_back(cs1_feature_descriptors);

  // ---------------------------------------------------------
  // Clique set 1 clusterings
  point_cloud_clustering::KMeans* kmeans_cs1 = new point_cloud_clustering::KMeans(0.001, 2);
  std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > cs1_clusterings(1);
  cs1_clusterings[0].first = false; // false means cluster over ALL points (not just nodes)
  cs1_clusterings[0].second = kmeans_cs1;
  clique_set_clusterings.push_back(cs1_clusterings);

  rf_creator_3d_ = new RFCreator3D(node_feature_descriptors, clique_set_feature_descriptors,
      clique_set_clusterings);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
boost::shared_ptr<RandomField> PtCloudRFCreator::createRandomField(const sensor_msgs::PointCloud& pt_cloud,
                                                                   const vector<float>& labels)
{
  vector<unsigned int> uint_labels(labels.begin(), labels.end());
  return rf_creator_3d_->createRandomField(pt_cloud, uint_labels, true);
}

boost::shared_ptr<RandomField> PtCloudRFCreator::createRandomField(const sensor_msgs::PointCloud& pt_cloud)
{
  return rf_creator_3d_->createRandomField(pt_cloud);
}

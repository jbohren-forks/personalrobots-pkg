/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */
#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>

#include <point_cloud_clustering/single_link.h>
#include <descriptors_3d/bounding_box_raw.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

using namespace std;

void FilterNoise(const map<unsigned int, RandomField::Node*>& nodes, const map<unsigned int,
    unsigned int>& inferred_labels, double max_single_link_radius, string fnamed_filtered)
{
  unsigned int BACKGROUND_LABEL = 1000;
  unsigned int FILTERED_LABEL = 999;

  // Point cloud of the nodes
  sensor_msgs::PointCloud pt_cloud;
  pt_cloud.points.resize(inferred_labels.size());

  // Mapping of label --> list of INDICES in POINT CLOUD with label
  // (not using the Node's id)
  map<unsigned int, set<unsigned int> > label_groups;

  ofstream filtered_pc(fnamed_filtered.c_str());

  // -----------------------------
  // Populate pt_cloud, label_groups
  unsigned int curr_pt_idx = 0;
  for (map<unsigned int, unsigned int>::const_iterator iter_inferred_labels =
      inferred_labels.begin() ; iter_inferred_labels != inferred_labels.end() ; iter_inferred_labels++, curr_pt_idx++)
  {
    // populate point cloud
    unsigned int curr_node_id = iter_inferred_labels->first;
    const RandomField::Node* curr_node = nodes.find(curr_node_id)->second;
    pt_cloud.points[curr_pt_idx].x = curr_node->getX();
    pt_cloud.points[curr_pt_idx].y = curr_node->getY();
    pt_cloud.points[curr_pt_idx].z = curr_node->getZ();

    // populate label-->[point indices] map (for NON-background labels only)
    unsigned int curr_label = iter_inferred_labels->second;
    if (curr_label == BACKGROUND_LABEL)
    {
      filtered_pc << pt_cloud.points[curr_pt_idx].x << " " << pt_cloud.points[curr_pt_idx].y << " "
          << pt_cloud.points[curr_pt_idx].z << " " << BACKGROUND_LABEL << endl;
    }
    else
    {
      if (label_groups.count(curr_label) == 0)
      {
        label_groups[curr_label] = set<unsigned int> ();
      }
      label_groups[curr_label].insert(curr_pt_idx);
    }
  }

  // -----------------------------
  // Create kdtree of point cloud
  cloud_kdtree::KdTreeANN pt_cloud_kdtree(pt_cloud);

  // -----------------------------
  // Create a map of [cluster volume] --> pair(cluster label, vector<point indices>)
  // -1 to indicate compute only computing dimensions of REGIONs
  map<float, pair<unsigned int, vector<int> > > ordered_cluster_volumes;
  BoundingBoxRaw bbox_descriptor(-1);
  point_cloud_clustering::SingleLink single_link(max_single_link_radius);
  cv::Vector<const vector<int>*> interest_region(1);
  cv::Vector<cv::Vector<float> > bbox_result;
  for (map<unsigned int, set<unsigned int> >::iterator iter_label_groups = label_groups.begin() ; iter_label_groups
      != label_groups.end() ; iter_label_groups++)
  {
    unsigned int curr_label = iter_label_groups->first;

    // Do single link clustering over the points that have curr_label
    std::map<unsigned int, std::vector<int> > created_clusters;
    single_link.cluster(pt_cloud, pt_cloud_kdtree, iter_label_groups->second, created_clusters);

    // For each group of clusters, record its volume in 3d space
    for (map<unsigned int, vector<int> >::iterator iter_created_clusters = created_clusters.begin() ; iter_created_clusters
        != created_clusters.end() ; iter_created_clusters++)
    {
      // specify the cluster to compute dimensions for
      vector<int>& cluster_pt_indices = iter_created_clusters->second;
      interest_region[0] = &cluster_pt_indices;

      // compute dimensions
      bbox_descriptor.compute(pt_cloud, pt_cloud_kdtree, interest_region, bbox_result);

      // compute volume
      if (bbox_result[0].size() == 0)
      {
        ROS_FATAL("No bbox results.  this should never happen");
        abort();
      }
      float volume = bbox_result[0][0] * bbox_result[0][1] * bbox_result[0][2];
      ROS_INFO("Label: %u  Volume: %f", curr_label, volume);

      // associate the cluster volume with its label and point indices within the cluster
      ordered_cluster_volumes[volume] = pair<unsigned int, vector<int> > (curr_label,
          cluster_pt_indices);
    }
  }

  bool is_first = true;
  for (map<float, pair<unsigned int, vector<int> > >::reverse_iterator iter_ordered_cluster_volumes =
      ordered_cluster_volumes.rbegin() ; iter_ordered_cluster_volumes
      != ordered_cluster_volumes.rend() ; iter_ordered_cluster_volumes++)
  {
    unsigned int label_of_cluster = iter_ordered_cluster_volumes->second.first;
    vector<int>& curr_pt_indices = iter_ordered_cluster_volumes->second.second;
    for (unsigned int i = 0 ; i < curr_pt_indices.size() ; i++)
    {
      filtered_pc << pt_cloud.points[curr_pt_indices[i]].x << " "
          << pt_cloud.points[curr_pt_indices[i]].y << " " << pt_cloud.points[curr_pt_indices[i]].z;

      // only print label for biggest cluster
      if (is_first)
      {
        filtered_pc << " " << label_of_cluster << endl;
      }
      else
      {
        filtered_pc << " " << FILTERED_LABEL << endl;
      }
    }
    is_first = false;
  }
  filtered_pc.close();

}

int main(int argc, char *argv[])
{
  unsigned int NBR_CLIQUE_SETS = 2;

  // ----------------------------
  if (argc != 4)
  {
    ROS_WARN("%s usage: <random field basename> <m3n model basename> <classified_basename>", argv[0]);
    return -1;
  }

  char* rf_basename = argv[1];
  char* m3n_model_basename = argv[2];
  string classified_basename(argv[3]);

  // ----------------------------------------------------------
  // Load random field
  RandomField testing_rf(NBR_CLIQUE_SETS);
  testing_rf.loadRandomField(rf_basename);

  // ----------------------------------------------------------
  // Load M3N model
  M3NModel m3n_model;
  if (m3n_model.loadFromFile(m3n_model_basename) < 0)
  {
    ROS_ERROR("Could not load M3N model");
    return -1;
  }

  // ----------------------------------------------------------
  // Classify
  ROS_INFO("Starting to classify...");
  map<unsigned int, unsigned int> inferred_labels;
  if (m3n_model.infer(testing_rf, inferred_labels) < 0)
  {
    ROS_ERROR("could not do inference");
    return -1;
  }
  ROS_INFO("Classified results");

  ROS_INFO("Saving node features...");
  testing_rf.updateLabelings(inferred_labels);
  string fname_node_features = classified_basename;
  fname_node_features.append("_classified.node_features");
  testing_rf.saveNodeFeatures(fname_node_features);
  ROS_INFO("done");

  ROS_INFO("Saving top cluster...");
  // 1 inch = 0.0254 m
  string fname_filtered_cloud = classified_basename;
  fname_filtered_cloud.append("_classified.xyz_label");
  //FilterNoise(testing_rf.getNodesRandomFieldIDs(), inferred_labels, 0.0254, fname_filtered_cloud);
  ROS_INFO("done");

  return 0;
}

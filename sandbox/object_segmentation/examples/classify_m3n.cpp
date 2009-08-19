/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>

#include <object_segmentation/util/rf_creator_3d.h>

extern RFCreator3D* initRFCreator();

extern int loadPointCloud(string filename,
                          unsigned int nbr_cols,
                          sensor_msgs::PointCloud& pt_cloud,
                          vector<unsigned int>& labels);

int main()
{
  unsigned int NUMBER_OF_COLUMNS = 5;

  // ----------------------------------------------------------
  // Load point cloud from file
  sensor_msgs::PointCloud pt_cloud;
  vector<unsigned int> labels;
  loadPointCloud("pt_cloud_260.xyz_label_conf", NUMBER_OF_COLUMNS, pt_cloud, labels);

  // ----------------------------------------------------------
  // Create random field
  RFCreator3D* rf_creator = initRFCreator();
  boost::shared_ptr<RandomField> testing_rf = rf_creator->createRandomField(pt_cloud);

  // ----------------------------------------------------------
  // Load M3N model
  M3NModel m3n_model;
  if (m3n_model.loadFromFile("m3n_models/2cs_pn_potts") < 0)
  {
    ROS_ERROR("Could not load M3N model");
    return -1;
  }

  // ----------------------------------------------------------
  // Classify
  ROS_INFO("Starting to classify...");
  map<unsigned int, unsigned int> inferred_labels;
  if (m3n_model.infer(*testing_rf, inferred_labels) < 0)
  {
    ROS_ERROR("could not do inference");
    return -1;
  }
  testing_rf->updateLabelings(inferred_labels);
  testing_rf->saveNodeFeatures("classified_results.node_features");
  ROS_INFO("Classified results");

  /*
   map<unsigned int, set<unsigned int> > label_groups; // label-->[list of indices]
   for (map<unsigned int, unsigned int>::iterator iter_inferred_labels = inferred_labels.begin() ; iter_inferred_labels
   != inferred_labels.end() ; iter_inferred_labels++)
   {
   unsigned int curr_idx = iter_inferred_labels->first;
   unsigned int curr_label = iter_inferred_labels->second;
   if (label_groups.count(curr_label) == 0)
   {
   label_groups[curr_label] = set<unsigned int> ();
   }
   label_groups[curr_label].insert(curr_idx);
   }

   cloud_kdtree::KdTreeANN pt_cloud_kdtree(pt_cloud);

   unsigned int seed = 0;

   point_cloud_clustering::SingleLink single_link(0.3048);
   for (map<unsigned int, set<unsigned int> >::iterator iter_label_groups = label_groups.begin() ; iter_label_groups
   != label_groups.end() ; iter_label_groups++)
   {
   std::map<unsigned int, std::vector<int> > created_clusters;
   single_link.cluster(pt_cloud, pt_cloud_kdtree, iter_label_groups->second, created_clusters);

   for (map<unsigned int, vector<int> >::iterator iter_created_clusters = created_clusters.begin() ; iter_created_clusters
   != created_clusters.end() ; iter_created_clusters++)
   {
   unsigned int curr_cluster_label = seed + iter_created_clusters->first;
   vector<int>& curr_pt_indices = iter_created_clusters->second;
   for (unsigned int i = 0 ; i < curr_pt_indices.size() ; i++)
   {
   cout << pt_cloud.pts[curr_pt_indices[i]].x << " " << pt_cloud.pts[curr_pt_indices[i]].y
   << " " << pt_cloud.pts[curr_pt_indices[i]].z << " " << curr_cluster_label << endl;
   }
   }

   seed += created_clusters.size();
   }
   */
  return 0;
}

/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */

#include <robot_msgs/PointCloud.h>

#include <functional_m3n/example/pt_cloud_rf_creator.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

// --------------------------------------------------------------
/*!
 * \brief Creates PointCloud datastructure from file
 *
 * File format: x y z label 2
 */
// --------------------------------------------------------------
int loadPointCloud(string filename,
                   unsigned int nbr_cols,
                   robot_msgs::PointCloud& pt_cloud,
                   vector<float>& labels)
{
  // ----------------------------------------------
  // Open file
  ifstream infile(filename.c_str());
  if (infile.is_open() == false)
  {
    ROS_ERROR("Could not open filename: %s", filename.c_str());
    return -1;
  }

  // ----------------------------------------------
  // Count number of lines in the file
  unsigned int nbr_samples = 0;
  char line[256];
  while (infile.getline(line, 256))
  {
    nbr_samples++;
  }

  infile.clear();
  infile.seekg(ios::beg);

  // ----------------------------------------------
  // Resize containers appropriately
  pt_cloud.pts.resize(nbr_samples);
  labels.resize(nbr_samples);

  // ----------------------------------------------
  // Read file
  // file format: x y z label 2
  int tempo;
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> pt_cloud.pts[i].x;
    infile >> pt_cloud.pts[i].y;
    infile >> pt_cloud.pts[i].z;
    unsigned int col = 3;
    if (col < nbr_cols)
    {
      infile >> labels[i];
      col++;
    }
    if (col < nbr_cols) infile >> tempo;
  }

  infile.close();
  return 0;
}

int main()
{
  // ----------------------------------------------------------
  // Load point cloud from file
  robot_msgs::PointCloud pt_cloud;
  unsigned int nbr_cols = 5;
  vector<float> labels;
  loadPointCloud("pt_cloud_260.xyz_label_conf", nbr_cols, pt_cloud, labels);

  // ----------------------------------------------------------
  // Create random field
  PtCloudRFCreator rf_creator;
  RandomField* testing_rf = rf_creator.createRandomField(pt_cloud);

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
  return 0;
}

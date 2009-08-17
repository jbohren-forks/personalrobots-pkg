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


int main(int argc, char *argv[])
{
  unsigned int NBR_CLIQUE_SETS = 2;

  // ----------------------------
  if (argc != 3)
  {
    ROS_WARN("%s usage: <random field basename> <m3n model basename>", argv[0]);
    return -1;
  }

  char* rf_basename = argv[1];
  char* m3n_model_basename = argv[2];

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
  testing_rf.updateLabelings(inferred_labels);
  testing_rf.saveNodeFeatures("classified_results.node_features");
  ROS_INFO("Classified results");

  return 0;
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <functional_m3n/random_field.h>

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField::RandomField()
{
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField::~RandomField()
{
  clear();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void RandomField::clear()
{
  // Free nodes (using random field id)
  for (map<unsigned int, Node*>::iterator iter_rf_nodes = rf_nodes_.begin() ; iter_rf_nodes
      != rf_nodes_.end() ; iter_rf_nodes++)
  {
    delete iter_rf_nodes->second;
  }

  // Free nodes (using sensor id)
  for (map<unsigned int, Node*>::iterator iter_sensor_nodes = sensor_nodes_.begin() ; iter_sensor_nodes
      != sensor_nodes_.end() ; iter_sensor_nodes++)
  {
    delete iter_sensor_nodes->second;
  }

  // Free cliques in each clique set
  for (unsigned int i = 0 ; i < clique_sets_.size() ; i++)
  {
    for (map<unsigned int, Clique*>::iterator iter_cliques = clique_sets_[i].begin() ; iter_cliques
        != clique_sets_[i].end() ; iter_cliques++)
    {
      delete iter_cliques->second;
    }
  }

  // Empty data structures
  rf_nodes_.clear();
  sensor_nodes_.clear();
  clique_sets_.clear();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
unsigned int RandomField::getLabelFromSensorID(const unsigned int sensor_id) const
{
#if ROS_DEBUG
  // Verify sensor id exists
  if (sensor_id == RandomField::UNKNOWN_SENSOR_ID || sensor_nodes_.count(sensor_id) == 0)
  {
    ROS_ERROR("Could not find sensor_id: %u", sensor_id);
    return UNKNOWN_LABEL;
  }
#endif

  return sensor_nodes_.find(sensor_id)->second->getLabel(); // sensor_nodes[sensor_id]->getLabel();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RandomField::updateLabelings(const map<unsigned int, unsigned int>& new_labeling)
{
  map<unsigned int, unsigned int>::const_iterator iter_new_labeling;

#if ROS_DEBUG
  // Ensure the number of nodes equals the new labeling mapping
  if (rf_nodes_.size() != new_labeling.size())
  {
    ROS_ERROR("Inconsistent number of nodes (%u) with mapping size (%u)", rf_nodes_.size(), new_labeling.size());
    return -1;
  }

  // Ensure the keys (node id) in new_labeling exist in this RandomField
  for (iter_new_labeling = new_labeling.begin(); iter_new_labeling != new_labeling.end(); iter_new_labeling++)
  {
    if (rf_nodes_.count(iter_new_labeling->first) == 0)
    {
      ROS_ERROR("Unknown node id from the map: %u", iter_new_labeling->first);
      return -1;
    }
  }
#endif

  // -------------------------------------------------------
  // Update node labels
  for (iter_new_labeling = new_labeling.begin(); iter_new_labeling != new_labeling.end() ; iter_new_labeling++)
  {
    (rf_nodes_[iter_new_labeling->first])->setLabel(iter_new_labeling->second);
  }

  // -------------------------------------------------------
  // Update label information in each clique in the clique sets
  map<unsigned int, Clique*>::iterator iter_cliques;
  Clique* curr_clique = NULL;
  for (unsigned int i = 0 ; i < clique_sets_.size() ; i++)
  {
    map<unsigned int, Clique*>& curr_cs = clique_sets_[i];
    for (iter_cliques = curr_cs.begin(); iter_cliques != curr_cs.end() ; iter_cliques++)
    {
      curr_clique = iter_cliques->second;
      curr_clique->updateLabels(new_labeling);
    }
  }

  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void RandomField::create(const robot_msgs::PointCloud& primitives, cloud_kdtree::KdTree& data_kdtree)

{
  createNodes(primitives, data_kdtree);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void RandomField::createNodes(const robot_msgs::PointCloud& primitives, cloud_kdtree::KdTree& data_kdtree)

{
  unsigned int poop = 1;

  unsigned int LABEL10 = 10;
  unsigned int LABEL69 = 69;

  double curvature = 0.0;
  Eigen::Vector4d plane_parameters;

  Node* new_node = NULL;

  unsigned int nbr_primitives = primitives.pts.size();
  vector<int> neighbor_indices;
  vector<float> neighbor_distances;
  for (unsigned int i = 0 ; i < nbr_primitives ; i++)
  {
    const robot_msgs::Point32& curr_pt = primitives.pts[i];

    neighbor_indices.clear();
    neighbor_distances.clear();
    if (data_kdtree.radiusSearch(curr_pt, 0.15, neighbor_indices, neighbor_distances))
    {
      cloud_geometry::nearest::computePointNormal(primitives, neighbor_indices, plane_parameters, curvature);

      if (curr_pt.z > 1.0) {
        new_node = new Node(curr_pt.x, curr_pt.y, curr_pt.z, poop, LABEL10);
      } else {
        new_node = new Node(curr_pt.x, curr_pt.y, curr_pt.z, poop, LABEL69);
      }
      rf_nodes_[poop] = new_node;
      poop++;

      vector<const FeatureDescriptor*>& curr_descrips = new_node->features_;
      FeatureDescriptor* blah = new FeatureDescriptor(curr_pt.z);
      curr_descrips.resize(1);
      curr_descrips[0] = blah;
      new_node->feature_vals_ = blah->feature_vals.data();
      new_node->nbr_feature_vals_ = 2;
    }
  }

  // TODO: after each created feature, call vectorizeFeatureVals()
}

// -----------------------------------------------------------------------------------------------------------
// RandomField::GenericClique, RandomField::Node, RandomField::Clique definitions below
// -----------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField::GenericClique::~GenericClique()
{
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField::Node::Node(const double x,
                        const double y,
                        const double z,
                        const unsigned int rf_id,
                        unsigned int label,
                        unsigned int sensor_id)
{
  x_ = x;
  y_ = y;
  z_ = z;
  rf_id_ = rf_id;

  label_ = label;
  sensor_id_ = sensor_id;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField::Clique::Clique(const unsigned int rf_id)
{
  rf_id_ = rf_id;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void RandomField::Clique::addNode(const Node& new_node)
{
  // Update centroid
  double prev_order = static_cast<double> (node_ids_.size());
  double new_order = prev_order + 1.0;
  x_ = (x_ * prev_order + new_node.getX()) / new_order;
  y_ = (y_ * prev_order + new_node.getY()) / new_order;
  z_ = (z_ * prev_order + new_node.getZ()) / new_order;

  // Add node id to list
  node_ids_.push_back(new_node.getRandomFieldID());

  // Add label->node_id to mapping
  if (labels_to_node_ids_.count(new_node.getLabel()) == 0)
  {
    labels_to_node_ids_[new_node.getLabel()] = list<unsigned int> ();
  }
  labels_to_node_ids_[new_node.getLabel()].push_back(new_node.getRandomFieldID());
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RandomField::Clique::updateLabels(const map<unsigned int, unsigned int>& node_labels)
{
  list<unsigned int>::iterator iter_node_ids;
  unsigned int curr_node_id = 0;

#if ROS_DEBUG
  // Verify labeling contains each node id contained in this Clique
  for (iter_node_ids = node_ids_.begin(); iter_node_ids != node_ids_.end(); iter_node_ids++)
  {
    curr_node_id = *iter_node_ids;
    if (node_labels.count(curr_node_id) == 0)
    {
      ROS_ERROR("Labeling map does not contain correct node ids");
      return -1;
    }
  }
#endif

  // Update label --> node_id mapping (labels_to_node_ids_)
  labels_to_node_ids_.clear();
  unsigned int curr_label = 0;
  for (iter_node_ids = node_ids_.begin(); iter_node_ids != node_ids_.end() ; iter_node_ids++)
  {
    curr_node_id = *iter_node_ids;
    curr_label = node_labels.find(curr_node_id)->second; // labeling[curr_node_id]
    if (labels_to_node_ids_.count(curr_label) == 0)
    {
      labels_to_node_ids_[curr_label] = list<unsigned int> ();
    }
    labels_to_node_ids_[curr_label].push_back(curr_node_id);
  }
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RandomField::Clique::getModeLabels(unsigned int& mode1_label,
                                       unsigned int& mode1_count,
                                       unsigned int& mode2_label,
                                       unsigned int& mode2_count,
                                       map<unsigned int, unsigned int>* tempo_labeling)
{
  mode1_label = 0;
  mode1_count = 0;
  mode2_label = 0;
  mode2_count = 0;

  map<unsigned int, list<unsigned int> > tempo_labels_to_node_ids;

  // -------------------------------------------------------
  // Determine a mapping from label->[node_ids].
  // Recalculate if using temporary label information,
  // otherwise use internal information
  map<unsigned int, list<unsigned int> >* labels_to_node_ids = NULL;
  if (tempo_labeling != NULL)
  {
    // populate temporary mapping: temporary_label --> [node ids]
    unsigned int curr_node_id = 0;
    unsigned int curr_tempo_node_label = 0;
    for (list<unsigned int>::iterator iter_node_ids = node_ids_.begin() ; iter_node_ids != node_ids_.end() ; iter_node_ids++)
    {
      curr_node_id = *iter_node_ids;

      // ---------
      // Get the node's temporary label
      if (tempo_labeling->count(curr_node_id) == 0)
      {
        ROS_ERROR("Could not find node id %u in the temporary labeling", curr_node_id);
        return -1;
      }
      curr_tempo_node_label = (*tempo_labeling)[curr_node_id];

      // ---------
      // Add node temporary label's list
      if (tempo_labels_to_node_ids.count(curr_tempo_node_label) == 0)
      {
        tempo_labels_to_node_ids[curr_tempo_node_label] = list<unsigned int> ();
      }
      tempo_labels_to_node_ids[curr_tempo_node_label].push_back(curr_node_id);
    }

    labels_to_node_ids = &tempo_labels_to_node_ids;
  }
  else
  {
    labels_to_node_ids = &labels_to_node_ids_;
  }

  // -------------------------------------------------------
  // Iterate over each label, compare each's number of associated nodes and update modes appropriately
  unsigned int curr_count = 0;
  for (map<unsigned int, list<unsigned int> >::iterator iter = labels_to_node_ids->begin() ; iter
      != labels_to_node_ids->end() ; iter++)
  {
    list<unsigned int>& curr_node_list = iter->second;
    curr_count = curr_node_list.size();

    // Update mode 1 if necessary
    if (curr_count > mode1_count)
    {
      // shift mode 1 to second place
      mode2_label = mode1_label;
      mode2_count = mode1_count;

      // update mode 1
      mode1_label = iter->first; // label
      mode1_count = curr_count;
    }
    // Update mode 2 if necessary
    else if (curr_count > mode2_count)
    {
      mode2_label = iter->first; // label
      mode2_count = curr_count;
    }
  }

  return 0;
}

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
/* See function definition */
// --------------------------------------------------------------
RandomField::RandomField(unsigned int nbr_clique_sets)
{
  clique_sets_.resize(nbr_clique_sets);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RandomField::~RandomField()
{
  clear();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void RandomField::clear()
{
  // Free nodes (using random field id)
  for (map<unsigned int, Node*>::iterator iter_rf_nodes = rf_nodes_.begin() ; iter_rf_nodes
      != rf_nodes_.end() ; iter_rf_nodes++)
  {
    delete iter_rf_nodes->second;
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
  clique_sets_.clear();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int RandomField::updateLabelings(const map<unsigned int, unsigned int>& new_labeling)
{
  map<unsigned int, unsigned int>::const_iterator iter_new_labeling;

#if DEBUG
  // Ensure the number of nodes equals the new labeling mapping
  if (rf_nodes_.size() != new_labeling.size())
  {
    ROS_ERROR("Inconsistent number of nodes (%u) with mapping size (%u)", rf_nodes_.size(), new_labeling.size());
    return -1;
  }

  // Ensure the keys (node id) in new_labeling exist in this RandomField
  for (iter_new_labeling = new_labeling.begin(); iter_new_labeling != new_labeling.end() ; iter_new_labeling++)
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
/* See function definition */
// --------------------------------------------------------------
const RandomField::Node* RandomField::createNode(const boost::shared_array<const float> feature_vals,
                                                 const unsigned int nbr_feature_vals,
                                                 unsigned int label,
                                                 float x,
                                                 float y,
                                                 float z)
{
  unsigned int unique_id = rf_nodes_.size();
  while (rf_nodes_.count(unique_id) != 0)
  {
    unique_id++;
  }
  return createNode(unique_id, feature_vals, nbr_feature_vals, label, x, y, z);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
const RandomField::Node* RandomField::createNode(const unsigned int node_id,
                                                 const boost::shared_array<const float> feature_vals,
                                                 const unsigned int nbr_feature_vals,
                                                 unsigned int label,
                                                 float x,
                                                 float y,
                                                 float z)
{
#if DEBUG
  // verify features are valid
  if (feature_vals.get() == NULL || nbr_feature_vals == 0)
  {
    ROS_ERROR("Invalid features for node");
    return NULL;
  }

  // verify clique id doesnt already eixt
  if (rf_nodes_.count(node_id) != 0)
  {
    ROS_ERROR("Cannot add node to random field b/c id %u already exists", node_id);
    return NULL;
  }
#endif

  RandomField::Node* new_node = new RandomField::Node(node_id, label);
  new_node->setFeatures(feature_vals, nbr_feature_vals);
  new_node->setXYZ(x, y, z);
  rf_nodes_[node_id] = new_node;
  return new_node;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
const RandomField::Clique* RandomField::createClique(const unsigned int clique_set_idx,
                                                     const list<const RandomField::Node*>& nodes,
                                                     const boost::shared_array<const float> feature_vals,
                                                     const unsigned int nbr_feature_vals,
                                                     float x,
                                                     float y,
                                                     float z)
{
#if DEBUG
  if (clique_set_idx > clique_sets_.size())
  {
    ROS_ERROR("RandomField::createClique clique_set_idx %u exceeds boundary %u", clique_set_idx, clique_sets_.size());
    return NULL;
  }
#endif

  map<unsigned int, RandomField::Clique*>& clique_set = clique_sets_[clique_set_idx];

  // generate unique clique id
  unsigned int unique_id = clique_set.size();
  while (clique_set.count(unique_id) != 0)
  {
    unique_id++;
  }
  return createClique(unique_id, clique_set_idx, nodes, feature_vals, nbr_feature_vals, x, y, z);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
const RandomField::Clique* RandomField::createClique(const unsigned int clique_id,
                                                     const unsigned int clique_set_idx,
                                                     const list<const RandomField::Node*>& nodes,
                                                     const boost::shared_array<const float> feature_vals,
                                                     const unsigned int nbr_feature_vals,
                                                     float x,
                                                     float y,
                                                     float z)
{
  list<const RandomField::Node*>::const_iterator iter_nodes;

#if DEBUG
  // verify features are valid
  if (feature_vals.get() == NULL || nbr_feature_vals == 0)
  {
    ROS_ERROR("Invalid features for clique %u", clique_id);
    return NULL;
  }

  // verify the clique set index is within bounds
  if (clique_set_idx > clique_sets_.size())
  {
    ROS_ERROR("RandomField::createClique clique_set_idx %u exceeds boundary %u", clique_set_idx, clique_sets_.size());
    return NULL;
  }

  // verify clique id doesnt already exist
  if (clique_sets_[clique_set_idx].count(clique_id) != 0)
  {
    ROS_ERROR("Cannot add clique to cs %u b/c id %u already exists", clique_set_idx, clique_id);
    return NULL;
  }

  // verify node ids are contained in this RandomField
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    if (rf_nodes_.count((*iter_nodes)->getRandomFieldID()) == 0)
    {
      ROS_ERROR("Could not add clique %u that contains node %u not in random field", clique_id, (*iter_nodes)->getRandomFieldID());
      return NULL;
    }
  }
#endif

  // instantiate clique
  RandomField::Clique* new_clique = new RandomField::Clique(clique_id);
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    new_clique->addNode(**iter_nodes);
  }
  new_clique->setFeatures(feature_vals, nbr_feature_vals);
  new_clique->setXYZ(x, y, z);

  // add clique to map container
  map<unsigned int, RandomField::Clique*>& clique_set = clique_sets_[clique_set_idx];
  clique_set[clique_id] = new_clique;
  return new_clique;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int RandomField::saveNodeFeatures(string filename) const
{
  ofstream file_out(filename.c_str());
  if (file_out.is_open() == false)
  {
    ROS_ERROR("Could not open requested %s to save node features to", filename.c_str());
    return -1;
  }

  file_out << "# File format: x y z node_id label nbr_features [features]" << endl;

  for (map<unsigned int, RandomField::Node*>::const_iterator iter_nodes = rf_nodes_.begin() ; iter_nodes
      != rf_nodes_.end() ; iter_nodes++)
  {
    const RandomField::Node* curr_node = iter_nodes->second;

    // x y z node_id label nbr_features
    file_out << curr_node->getX() << " " << curr_node->getY() << " " << curr_node->getZ() << " "
        << curr_node->getRandomFieldID() << " " << curr_node->getLabel() << " "
        << curr_node->getNumberFeatureVals();

    // [features]
    const boost::shared_array<const float> curr_feats = curr_node->getFeatureVals();
    for (unsigned int feature_idx = 0 ; feature_idx < curr_node->getNumberFeatureVals() ; feature_idx++)
    {
      // TODO set precision
      file_out << " " << curr_feats[feature_idx];
    }
    file_out << endl;
  }
  file_out.close();
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int RandomField::saveCliqueFeatures(string basename) const
{
  const unsigned int nbr_clique_sets = clique_sets_.size();
  for (unsigned int cs_idx = 0 ; cs_idx < nbr_clique_sets ; cs_idx++)
  {
    // Generate filename for current clique set's features
    stringstream ss_curr_filename;
    ss_curr_filename << basename << ".cs_" << cs_idx << "_features";
    string curr_filename = ss_curr_filename.str();

    // Open file
    ofstream file_out(curr_filename.c_str());
    if (file_out.is_open() == false)
    {
      ROS_ERROR("Could not open requested %s to save clique features to", curr_filename.c_str());
      return -1;
    }

    file_out << "# File format: x y z clique_set_idx clique_id nbr_features [features]" << endl;

    // Write to file: x y z clique_set_idx clique_id nbr_features [features]
    const map<unsigned int, Clique*>& cliques = clique_sets_[cs_idx];
    for (map<unsigned int, Clique*>::const_iterator iter_cliques = cliques.begin() ; iter_cliques
        != cliques.end() ; iter_cliques++)
    {
      const unsigned int curr_clique_id = iter_cliques->first;
      const RandomField::Clique* curr_clique = iter_cliques->second;

      // x y z clique_set_idx clique_id nbr_features
      file_out << curr_clique->getX() << " " << curr_clique->getY() << " " << curr_clique->getZ() << " "
          << cs_idx << " " << curr_clique_id << " " << curr_clique->getNumberFeatureVals();

      // [features]
      const boost::shared_array<const float> curr_feats = curr_clique->getFeatureVals();
      for (unsigned int feature_idx = 0 ; feature_idx < curr_clique->getNumberFeatureVals() ; feature_idx++)
      {
        // TODO set precision
        file_out << " " << curr_feats[feature_idx];
      }
      file_out << endl;
    }

    file_out.close();
  }
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int RandomField::saveRandomField(string basename) const
{
  string rf_fname = basename;
  rf_fname.append(".random_field");
  ofstream file_out(rf_fname.c_str());
  if (file_out.is_open() == false)
  {
    ROS_ERROR("Could not open requested %s to save random field to", rf_fname.c_str());
    return -1;
  }

  string node_features_fname = basename;
  node_features_fname.append(".node_features");
  if (saveNodeFeatures(node_features_fname) < 0)
  {
    file_out.close();
    return -1;
  }

  if (saveCliqueFeatures(basename) < 0)
  {
    file_out.close();
    return -1;
  }

  file_out << "# File format:" << endl;
  file_out << "# node_features_filename" << endl;
  file_out << "# " << endl;
  file_out << "# S=nbr_clique_sets" << endl;
  file_out << "# clique_set_0_features_filename" << endl;
  file_out << "# 0 P=nbr_cliques_in_set" << endl;
  file_out << "# 0 clique_id_0 nbr_nodes_in_clique [node ids]" << endl;
  file_out << "#    ..." << endl;
  file_out << "# 0 clique_id_P nbr_nodes_in_clique [node ids]" << endl;
  file_out << "#" << endl;
  file_out << "# ..." << endl;
  file_out << "# clique_set_(S-1)_features_filename" << endl;
  file_out << "# (S-1) P=nbr_cliques" << endl;
  file_out << "# (S-1) clique_id_0 nbr_nodes_in_clique [node ids]" << endl;
  file_out << "#    ..." << endl;
  file_out << "# (S-1) clique_id_P nbr_nodes_in_clique [node ids]" << endl;

  file_out << node_features_fname << endl;
  file_out << endl;
  file_out << clique_sets_.size() << endl;
  for (unsigned int i = 0 ; i < clique_sets_.size() ; i++)
  {
    const map<unsigned int, Clique*>& cliques = clique_sets_[i];

    // print clique-set feature filename
    stringstream ss_cs_fname;
    ss_cs_fname << basename << ".cs_" << i << "_features";
    file_out << ss_cs_fname.str() << endl;

    // clique-set-idx and number of cliques
    file_out << i << " " << cliques.size() << endl;

    // print each clique in the clique set
    for (map<unsigned int, Clique*>::const_iterator iter_cliques = cliques.begin() ; iter_cliques
        != cliques.end() ; iter_cliques++)
    {
      const list<unsigned int>& node_ids = iter_cliques->second->getNodeIDs();
      file_out << i << " " << iter_cliques->first << " " << node_ids.size();

      // print node ids in the clique
      for (list<unsigned int>::const_iterator iter_node_ids = node_ids.begin() ; iter_node_ids
          != node_ids.end() ; iter_node_ids++)
      {
        file_out << " " << *iter_node_ids;
      }
      file_out << endl;
    }

    file_out << endl;
  }

  file_out.close();
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int RandomField::loadRandomField(string basename)
{
  string rf_fname = basename;
  rf_fname.append(".random_field");
  ifstream file_rf(rf_fname.c_str());
  if (file_rf.is_open() == false)
  {
    ROS_ERROR("Could not open requested %s to load random field from", rf_fname.c_str());
    return -1;
  }

  clear();

  // skip header
  for (int i = 0 ; i < 16 ; i++)
  {
    file_rf.ignore(std::numeric_limits<int>::max(), '\n');
  }

  // ------------------------------------------
  // Load node features file
  // format: x y z node_id label nbr_features [features]
  char node_features_fname[512];
  file_rf.getline(node_features_fname, 512);
  ifstream file_node_features(node_features_fname);
  if (file_node_features.is_open() == false)
  {
    ROS_ERROR("Could not open node features to load: %s", node_features_fname);
    file_rf.close();
    return -1;
  }
  // skip header
  file_node_features.ignore(std::numeric_limits<int>::max(), '\n');
  string line;
  while (getline(file_node_features, line))
  {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    unsigned int node_id = 0;
    unsigned int node_label = RandomField::UNKNOWN_LABEL;
    unsigned int nbr_feature_vals = 0;

    istringstream iss(line);
    iss >> x;
    iss >> y;
    iss >> z;
    iss >> node_id;
    iss >> node_label;
    iss >> nbr_feature_vals;

    float* feature_vals = new float[nbr_feature_vals];
    for (unsigned int i = 0 ; i < nbr_feature_vals ; i++)
    {
      iss >> feature_vals[i];
    }
    boost::shared_array<const float> const_feature_vals(static_cast<const float*> (feature_vals));

    if (createNode(node_id, const_feature_vals, nbr_feature_vals, node_label, x, y, z) == NULL)
    {
      abort();
    }
  }
  file_node_features.close();

  // ------------------------------------------
  // Load clique set info
  unsigned int nbr_clique_sets = 0;
  file_rf >> nbr_clique_sets;
  file_rf.ignore(std::numeric_limits<int>::max(), '\n'); // finish the line

  // For each clique-set, read its features and clique<->node membership info
  clique_sets_.resize(nbr_clique_sets);
  for (unsigned int cs_idx = 0 ; cs_idx < nbr_clique_sets ; cs_idx++)
  {
    // --------------------------
    // Load clique features
    // format: x y z clique_set_idx clique_id nbr_features [features]
    char clique_features_fname[512];
    file_rf.getline(clique_features_fname, 512);
    ifstream file_clique_features(clique_features_fname);
    if (file_clique_features.is_open() == false)
    {
      ROS_ERROR("Could not open clique features to load: %s", clique_features_fname);
      file_rf.close();
      return -1;
    }

    // skip header
    file_clique_features.ignore(std::numeric_limits<int>::max(), '\n');
    string line;
    while (getline(file_clique_features, line))
    {
      float x = 0.0;
      float y = 0.0;
      float z = 0.0;
      unsigned int read_cs_idx = 0;
      unsigned int clique_id = 0;
      unsigned int nbr_feature_vals = 0;

      istringstream iss(line);
      iss >> x;
      iss >> y;
      iss >> z;
      iss >> read_cs_idx;
      iss >> clique_id;
      iss >> nbr_feature_vals;

      float* feature_vals = new float[nbr_feature_vals];
      for (unsigned int i = 0 ; i < nbr_feature_vals ; i++)
      {
        iss >> feature_vals[i];
      }
      boost::shared_array<const float> const_feature_vals(static_cast<const float*> (feature_vals));

      if (cs_idx != read_cs_idx)
      {
        abort();
      }

      Clique* new_clique = new Clique(clique_id);
      new_clique->setXYZ(x, y, z);
      new_clique->setFeatures(const_feature_vals, nbr_feature_vals);
      clique_sets_[cs_idx][clique_id] = new_clique;
    }
    file_clique_features.close();

    // --------------------------
    // Read node membership for each clique
    // Format:
    //   cs_idx_i P=nbr_cliques_in_set
    //     ... loop over j < P...
    //   cs_idx_i clique_j_id nbr_nodes_in_clique [node ids]
    unsigned int read_cs_idx = 0;
    unsigned int read_nbr_cliques = 0;
    file_rf >> read_cs_idx;
    file_rf >> read_nbr_cliques;

    // verify reading the clique for the correct clique-set
    if (cs_idx != read_cs_idx)
    {
      abort();
    }
    // verify the read number of cliques matches what was read from feature file
    if (clique_sets_[cs_idx].size() != read_nbr_cliques)
    {
      abort();
    }

    // Iterate over each clique in the clique-set
    for (unsigned int clique_index = 0 ; clique_index < clique_sets_[cs_idx].size() ; clique_index++)
    {
      unsigned int read_cs_idx2 = 0;
      unsigned int clique_id = 0;
      unsigned int clique_order = 0;

      file_rf >> read_cs_idx2;
      file_rf >> clique_id;
      file_rf >> clique_order;

      // verify the read clique-set index and clique id exist
      if (cs_idx != read_cs_idx2)
      {
        abort();
      }
      if (clique_sets_[cs_idx].count(clique_id) == 0)
      {
        abort();
      }

      // read the node ids contained in each clique
      Clique* curr_clique = clique_sets_[cs_idx][clique_id];
      for (unsigned int node_index = 0 ; node_index < clique_order ; node_index++)
      {
        unsigned int node_id = 0;
        file_rf >> node_id;

        // verify it exists
        if (rf_nodes_.count(node_id) == 0)
        {
          abort();
        }
        curr_clique->addNode(*(rf_nodes_[node_id]));
      }
    }
    file_rf.ignore(std::numeric_limits<int>::max(), '\n'); // finish the line
    file_rf.ignore(std::numeric_limits<int>::max(), '\n'); // skip blank line
  }
  return 0;
}
// -----------------------------------------------------------------------------------------------------------
// RandomField::GenericClique, RandomField::Node, RandomField::Clique definitions below
// -----------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RandomField::GenericClique::GenericClique()
{
  id_ = 0;
  nbr_feature_vals_ = 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RandomField::GenericClique::~GenericClique()
{
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RandomField::Node::Node(const unsigned int rf_id, unsigned int label)
{
  id_ = rf_id;
  label_ = label;

  x_ = 0.0;
  y_ = 0.0;
  z_ = 0.0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RandomField::Clique::Clique(const unsigned int rf_id)
{
  id_ = rf_id;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void RandomField::Clique::addNode(const Node& new_node)
{
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
/* See function definition */
// --------------------------------------------------------------
int RandomField::Clique::updateLabels(const map<unsigned int, unsigned int>& node_labels)
{
  list<unsigned int>::iterator iter_node_ids;
  unsigned int curr_node_id = 0;

#if DEBUG
  // Verify labeling contains each node id contained in this Clique
  for (iter_node_ids = node_ids_.begin(); iter_node_ids != node_ids_.end() ; iter_node_ids++)
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
/* See function definition */
// --------------------------------------------------------------
int RandomField::Clique::getModeLabels(unsigned int& mode1_label,
                                       unsigned int& mode1_count,
                                       unsigned int& mode2_label,
                                       unsigned int& mode2_count,
                                       list<unsigned int>* mode1_node_ids,
                                       const map<unsigned int, unsigned int>* tempo_labeling) const
{
  mode1_label = RandomField::UNKNOWN_LABEL;
  mode1_count = RandomField::UNKNOWN_LABEL;
  mode2_label = RandomField::UNKNOWN_LABEL;
  mode2_count = RandomField::UNKNOWN_LABEL;

  map<unsigned int, list<unsigned int> > tempo_labels_to_node_ids;

  // -------------------------------------------------------
  // Determine a mapping from label->[node_ids].
  // Recalculate if using temporary label information,
  // otherwise use internal information
  const map<unsigned int, list<unsigned int> >* labels_to_node_ids = NULL;
  if (tempo_labeling != NULL)
  {
    // populate temporary mapping: temporary_label --> [node ids]
    unsigned int curr_node_id = 0;
    unsigned int curr_tempo_node_label = 0;
    for (list<unsigned int>::const_iterator iter_node_ids = node_ids_.begin() ; iter_node_ids
        != node_ids_.end() ; iter_node_ids++)
    {
      curr_node_id = *iter_node_ids;

      // ---------
      // Get the node's temporary label
#if DEBUG
      if (tempo_labeling->count(curr_node_id) == 0)
      {
        ROS_ERROR("Could not find node id %u in the temporary labeling", curr_node_id);
        return -1;
      }
#endif
      curr_tempo_node_label = tempo_labeling->find(curr_node_id)->second;

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
  for (map<unsigned int, list<unsigned int> >::const_iterator iter = labels_to_node_ids->begin() ; iter
      != labels_to_node_ids->end() ; iter++)
  {
    const list<unsigned int>& curr_node_list = iter->second;
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

  // -------------------------------------------------------
  // Save node ids that are labeled mode1_label if indicated
  if (mode1_node_ids != NULL)
  {
    *mode1_node_ids = labels_to_node_ids->find(mode1_label)->second;
  }

  return 0;
}

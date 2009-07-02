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
void RandomField::createNodes(const robot_msgs::PointCloud& primitives, cloud_kdtree::KdTree& data_kdtree)

{
  // Create one feature TYPE
  list<pair<unsigned int, Eigen::MatrixXd*> > created_features;
  list<pair<unsigned int, Eigen::MatrixXd*> >::iterator iter_created_features;
  LocalGeometry geometric_features;
  geometric_features.setData(&primitives, &data_kdtree);
  geometric_features.defineNeighborhood(0.15);
  geometric_features.useElevation();
  geometric_features.useNormalOrientation(0.0, 0.0, 1.0);
  geometric_features.useTangentOrientation(0.0, 0.0, 1.0);

  Eigen::MatrixXd* curr_features = NULL;

  Node* new_node = NULL;
  unsigned int unique_node_id = rf_nodes_.size();
  unsigned int nbr_primitives = primitives.pts.size();
  unsigned int nbr_feature_vals = 0;
  for (unsigned int i = 0 ; i < nbr_primitives ; i++)
  {
    created_features.clear();
    nbr_feature_vals = 0;

    // -------------------------------
    // Attempt to compute all features
    geometric_features.setInterestPoint(i);
    if (geometric_features.compute(&curr_features, false) == true)
    {
      created_features.push_back(pair<unsigned int, Eigen::MatrixXd*> (geometric_features.getResultSize(),
          curr_features));
      nbr_feature_vals += geometric_features.getResultSize();
    }
    else
    {
      continue;
    }

    // -------------------------------
    // Features successfully computed, now create node
    // Generate unique node id
    while (rf_nodes_.count(unique_node_id) != 0)
    {
      unique_node_id++;
    }

    new_node = new Node(primitives.pts[i].x, primitives.pts[i].y, primitives.pts[i].z, unique_node_id);
    new_node->nbr_feature_vals_ = nbr_feature_vals;
    new_node->feature_vals_ = static_cast<double*> (malloc(nbr_feature_vals * sizeof(double)));

    // Copy each feature from above
    unsigned int prev_length = 0;
    unsigned int curr_length = 0;
    for (iter_created_features = created_features.begin(); iter_created_features != created_features.end() ; iter_created_features++)
    {
      curr_length = iter_created_features->first;
      memcpy(new_node->feature_vals_ + prev_length, iter_created_features->second->data(), curr_length
          * sizeof(double));
      prev_length += curr_length;
      delete iter_created_features->second;
    }
  }
}

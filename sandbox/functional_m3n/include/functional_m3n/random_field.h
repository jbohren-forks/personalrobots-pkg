#ifndef __RANDOM_FIELD_H__
#define __RANDOM_FIELD_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <list>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <robot_msgs/PointCloud.h>

// 3rd party
#include <Eigen/Core>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;

// TODO TEMPO
class FeatureDescriptor
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureDescriptor(double f)
    {
      feature_vals[0] = f;
      feature_vals[1] = 1.0;
    }

    unsigned int getDimension()
    {
      return 2;
    }
    Eigen::Vector2d feature_vals;
};
// TODO TEMPO

// --------------------------------------------------------------
//* RandomField
/**
 * \brief Class to represent a random field over a point cloud
 *
 * Conceptually, nodes are the primitives things you wish to classify and
 * cliques describe regions/groups of primitives.
 */
// --------------------------------------------------------------
class RandomField
{

  public:
    const static unsigned int UNKNOWN_LABEL = 0;
    const static unsigned int UNKNOWN_SENSOR_ID = 0;

    // --------------------------------------------------------------
    /** \see definition below */
    // --------------------------------------------------------------
    class Node;

    // --------------------------------------------------------------
    /** \see definition below */
    // --------------------------------------------------------------
    class Clique;

    // --------------------------------------------------------------
    /*!
     * \brief Instantiates an empty Random Field
     */
    // --------------------------------------------------------------
    RandomField();

    // --------------------------------------------------------------
    /*!
     * \brief Destroys this Random Field
     */
    // --------------------------------------------------------------
    ~RandomField();

    // --------------------------------------------------------------
    /*!
     * \brief Clears all nodes and cliques contained in this random field.
     */
    // --------------------------------------------------------------
    void clear();

    // --------------------------------------------------------------
    /*!
     * \brief Returns mapping of ids from the sensor to associated primitives
     */
    // --------------------------------------------------------------
    inline const map<unsigned int, Node*>& getNodesSensorIDs() const
    {
      return sensor_nodes_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns mapping of ids assigned from this RandomField to associated primitives
     */
    // --------------------------------------------------------------
    inline const map<unsigned int, Node*>& getNodesRandomFieldIDs() const
    {
      return rf_nodes_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns vector of clique-sets, which are represented by a mapping from clique ids
     * assigned from this RandomField to associated cliques
     */
    // --------------------------------------------------------------
    inline const vector<map<unsigned int, Clique*> >& getCliqueSets() const
    {
      return clique_sets_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the label for a primitive for the specified sensor id
     */
    // --------------------------------------------------------------
    unsigned int getLabelFromSensorID(const unsigned int sensor_id) const;

    // --------------------------------------------------------------
    /*!
     * \brief Updates the labels for the nodes in this RandomField
     *
     * \warning This is a somewhat slow call as all label information contained
     * in the cliques also need to be updated
     *
     * \param new_labeling Mapping of node ids to their new labels
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int updateLabelings(const map<unsigned int, unsigned int>& new_labeling);

    // --------------------------------------------------------------
    /*!
     * \brief TODO: node feature params, clique construction, clique params
     */
    // --------------------------------------------------------------
    void create(const robot_msgs::PointCloud& primitives, cloud_kdtree::KdTree& data_kdtree);

  private:
    // ----- TODO -----------
    void createNodes(const robot_msgs::PointCloud& points, cloud_kdtree::KdTree& data_kdtree);
    void createCliqueSet(void* point_cloud, void* construction_params, void* feature_params);
    // ----- TODO -----------

    map<unsigned int, Node*> rf_nodes_; // using rf_id
    map<unsigned int, Node*> sensor_nodes_; // using sensor_id
    vector<map<unsigned int, Clique*> > clique_sets_;

  public:
    class GenericClique
    {
      public:
        virtual ~GenericClique() = 0;

        // --------------------------------------------------------------
        /*!
         * \brief Returns the x-coordinate
         */
        // --------------------------------------------------------------
        inline double getX() const
        {
          return x_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the y-coordinate
         */
        // --------------------------------------------------------------
        inline double getY() const
        {
          return y_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the z-coordinate
         */
        // --------------------------------------------------------------
        inline double getZ() const
        {
          return z_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the unique RandomField id of the node
         */
        // --------------------------------------------------------------
        inline unsigned int getRandomFieldID() const
        {
          return rf_id_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the features associated with this Node
         */
        // --------------------------------------------------------------
        inline const vector<const FeatureDescriptor*>& getFeatures() const
        {
          return features_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the values of the features contained in the descriptors in vector format
         */
        // --------------------------------------------------------------
        inline const double* getFeatureVals() const
        {
          return feature_vals_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the number of feature values in vector format
         */
        // --------------------------------------------------------------
        inline const unsigned int getNumberFeatureVals() const
        {
          return nbr_feature_vals_;
        }

      protected:
        unsigned int rf_id_;

        double x_;
        double y_;
        double z_;

        // TODO maintain these jointly
        vector<const FeatureDescriptor*> features_;
        double* feature_vals_;
        unsigned int nbr_feature_vals_;
    };

    // --------------------------------------------------------------
    //* Node
    /**
     * \brief Represents the primitive to classify
     */
    // --------------------------------------------------------------
    class Node: public GenericClique
    {
        // Allow RandomField to access the protected functions
        friend class RandomField;

      public:
        // --------------------------------------------------------------
        /*!
         * \brief Instantiate a node with the given parameters
         *
         * \param x The x-coordinate of the primitive
         * \param y The y-coordinate of the primitive
         * \param z The z-coordinate of the primitive
         * \param rf_id The unique id assigned to this node by the RandomField
         * \param label (Optional) The label of the primitive
         * \param sensor_id (Optional) A unique id to the primitive assigned by the sensor
         */
        // --------------------------------------------------------------
        Node(const double x, const double y, const double z, const unsigned int rf_id, unsigned int label =
            UNKNOWN_LABEL, unsigned int sensor_id = UNKNOWN_SENSOR_ID);

        // --------------------------------------------------------------
        /*!
         * \brief Returns the label of the node
         */
        // --------------------------------------------------------------
        inline unsigned int getLabel() const
        {
          return label_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the unique external sensor id of the node
         */
        // --------------------------------------------------------------
        inline unsigned int getSensorID() const
        {
          return sensor_id_;
        }

      protected:
        // --------------------------------------------------------------
        /*!
         * \brief Sets the label of this Node
         *
         * \param new_label The new label
         *
         * \warning Use with caution. Should afterwards call RandomField::Clique::updateLabels
         */
        // --------------------------------------------------------------
        inline void setLabel(const unsigned int new_label)
        {
          label_ = new_label;
        }

      private:
        unsigned int label_;
        unsigned int sensor_id_;
    };

    // --------------------------------------------------------------
    //* Node
    /**
     * \brief Represents a group of primitives
     */
    // --------------------------------------------------------------
    class Clique: public GenericClique
    {
        // Allow RandomField to access the protected functions
        friend class RandomField;

      public:
        // --------------------------------------------------------------
        /*!
         * \brief Instantiates a new Clique with given unique id from the RandomField
         *
         * \param rf_id Unique id from the owner RandomField
         */
        // --------------------------------------------------------------
        Clique(const unsigned int rf_id);

        // --------------------------------------------------------------
        /*!
         * \brief Returns the number of nodes contained in this Clique
         */
        // --------------------------------------------------------------
        inline unsigned int getOrder() const
        {
          return node_ids_.size();
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the node ids contained in this Clique
         */
        // --------------------------------------------------------------
        inline const list<unsigned int>& getNodeIDs() const
        {
          return node_ids_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the mapping of label to node ids with respective label in this Clique
         */
        // --------------------------------------------------------------
        inline const map<unsigned int, list<unsigned int> >& getLabelsToNodeIDs() const
        {
          return labels_to_node_ids_;
        }

        // --------------------------------------------------------------
        /*!
         * \brief Returns the mode and second mode labels contained in this Clique
         *
         * \param mode1_label Reference to store the mode label
         * \param mode1_count Reference to store the number of nodes with mode1_label
         * \param mode2_label Reference to store the 2nd mode label (may be UNKNOWN_LABEL)
         * \param mode2_count Reference to store the number of nodes with mode2_label
         * \param tempo_labeling (Optional) Instead of using internal label information to compute
         *                       modes, will act as if each contained node is labeled using this
         *                       map of node_id -> label.  Use NULL to not use.
         *
         * \return 0 on success, otherwise negative value
         */
        // --------------------------------------------------------------
        int getModeLabels(unsigned int& mode1_label,
                          unsigned int& mode1_count,
                          unsigned int& mode2_label,
                          unsigned int& mode2_count,
                          map<unsigned int, unsigned int>* tempo_labeling = NULL);

      protected:
        // --------------------------------------------------------------
        /*!
         * \brief Adds the given Node to this Clique
         *
         * \param new_node The new node to add to this Clique
         */
        // --------------------------------------------------------------
        void addNode(const Node& new_node);

        // --------------------------------------------------------------
        /*!
         * \brief Updates the information regarding what labels are contained in this Clique
         *
         * \param node_labels Mapping from node id to label. Map size can be bigger than clique order.
         *
         * \warning Use with caution.  Should be called after RandomField::Node::setLabel
         *
         * \return 0 on success, otherwise negative value on error.
         */
        // --------------------------------------------------------------
        int updateLabels(const map<unsigned int, unsigned int>& node_labels);

      private:
        list<unsigned int> node_ids_;
        map<unsigned int, list<unsigned int> > labels_to_node_ids_;
    };

};
#endif

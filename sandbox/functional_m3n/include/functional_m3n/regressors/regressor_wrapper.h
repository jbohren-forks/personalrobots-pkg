#ifndef __REGRESSOR_WRAPPER_H__
#define __REGRESSOR_WRAPPER_H__
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
#include <string>

using namespace std;

// --------------------------------------------------------------
//* RegressionTreeWrapper
/**
 * \brief Generic wrapper around various regressor/classification algorithms.
 *
 * Multi-class regression/classification can be handled by creating a "stacked"
 * sparse feature vector representation.
 *
 * Example: 3-class (A,B,C) 1-vs-all classification.
 * Assume the feature x has dimension 1 with ground truth label B.
 * Instead of training 3 classifiers f_A, f_B, f_C, where
 * the training samples for each classifier are
 *   (x,-1) -> f_A
 *   (x,+1) -> f_B
 *   (x,-1) -> f_C
 * we will use 1 classifier g with "stacked" feature vectors
 * that encapsulates this idea.  In this example, we create a
 * 3-dimensional vector (dim(x)=1) for each label, i.e.
 * non-zero in the dimension of the desired label.  So in the
 * above example, we train g with the features
 *   ([x,0,0], -1)
 *   ([0,x,0], +1)
 *   ([0,0,x], -1)
 * When predicting the class of a new feature y, we would evaluate
 *   g([y,0,0])  (score for class A)
 *   g([0,y,0])  (score for class B)
 *   g([0,0,y])  (score for class C)
 * and pick the class that had the biggest value.
 *
 * The stacked feature dimension is defined when instantiating
 * the inherited class.
 * This stacked feature representation is OPTIONAL.  Define stacked_feature_dim_
 * and the parameters in addTrainingSample() and predict() accordingly.
 */
// --------------------------------------------------------------
class RegressorWrapper
{
  public:
    typedef enum algorithm
    {
      // Default flag
      REGRESSION_NONE = 0,
      // Regression Tree (from OpenCV)
      REGRESSION_TREE = 1,
    } algorithm_t;

    virtual ~RegressorWrapper()
    {
    }
    ;

    // --------------------------------------------------------------
    /**
     * \brief Returns the type of regressor algorithm
     */
    // --------------------------------------------------------------
    algorithm_t getAlgorithmType() const
    {
      return algorithm_type_;
    }

    // --------------------------------------------------------------
    /**
     * \brief Clears the state of the regressor so it can be retrained
     */
    // --------------------------------------------------------------
    virtual void clear() = 0;

    // --------------------------------------------------------------
    /**
     * \brief Saves the trained regressor to file format
     *
     * \param basename The basename of the file to save the regressor and related files
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int saveToFile(const string& basename) = 0;

    // --------------------------------------------------------------
    /**
     * \brief Loads a previously saved, trained regressor from file
     *
     * \param basename The basename of the file of the saved regressor and related files
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int loadFromFile(const string& basename) = 0;

    // --------------------------------------------------------------
    /**
     * \brief Adds a (feature,target) data sample to train on
     *
     * \param feature_vec Pointer to array of features
     * \param length The number of values in feature_vec (should be equal to
     *               stacked_feature_dim_ if NOT using "stacked" representation)
     * \param start_idx The dimension to place the features in the "stacked"
     *                  feature representation (should be 0 if NOT using
     *                  "stacked" representation)
     * \param target The target response of the features
     *
     * \warning feature_vec should not be freed until train() has been called
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int addTrainingSample(const double* const feature_vec,
                                  const unsigned int length,
                                  const unsigned int start_idx,
                                  const double target) = 0;

    // --------------------------------------------------------------
    /**
     * \brief Trains the regressor using all the added training samples
     *
     * \return 0 on success, otherwise negative value on error.  If error,
     *         all previously added training samples are lost and must be
     *         re-added.
     */
    // --------------------------------------------------------------
    virtual int train() = 0;

    // --------------------------------------------------------------
    /**
     * \brief Predicts the value of the feature with this trained regressor/classifier
     *
     * \param feature_vec Pointer to array of features
     * \param length The number of values in feature_vec (should be equal to
     *               stacked_feature_dim_ if NOT using "stacked" representation)
     * \param start_idx The dimension to place the features in the "stacked"
     *                  feature representation (should be 0 if NOT using
     *                  "stacked" representation)
     * \param predicted_val The predicted/response value
     *
     * \warning No check is done to ensure start_idx+length <= stacked_featuer_dim_
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int predict(const double* const feature_vec,
                        const unsigned int length,
                        const unsigned int start_idx,
                        double& predicted_val) = 0;

  protected:
    algorithm_t algorithm_type_;
    unsigned int stacked_feature_dim_;
    bool trained_;
};

#endif

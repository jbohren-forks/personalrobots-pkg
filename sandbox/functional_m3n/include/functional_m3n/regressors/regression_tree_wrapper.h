#ifndef __REGRESSION_TREE_WRAPPER_H__
#define __REGRESSION_TREE_WRAPPER_H__
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
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <opencv/ml.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

#include <functional_m3n/regressors/regressor_wrapper.h>
#include <functional_m3n/regressors/regressor_params.h>

using namespace std;

// --------------------------------------------------------------
/*!
 * \file regression_tree_wrapper.h
 *
 * \brief Wrapper around the OpenCV regression tree implementation
 */
// --------------------------------------------------------------
class RegressionTreeWrapper: public RegressorWrapper
{
  public:
    // --------------------------------------------------------------
    /**
     * \brief Create regression tree with default parameters.
     *
     * See RegressionTreeWrapperParams for default values
     *
     * \param stacked_feature_dim The total (stacked) feature dimension used
     *                            by this regression tree
     *
     * \warning This regression tree is forever bound to these parameters
     */
    // --------------------------------------------------------------
    RegressionTreeWrapper(unsigned int stacked_feature_dim);

    // --------------------------------------------------------------
    /**
     * \brief Create regression tree with specified parameters.
     *
     * \param stacked_feature_dim The total (stacked) feature dimension used
     *                            by this regression tree
     * \param rtree_params Regression tree parameters
     *
     * \warning No check is done to ensure the parameters are valid
     * \warning This regression tree is forever bound to these parameters
     */
    // --------------------------------------------------------------
    RegressionTreeWrapper(unsigned int stacked_feature_dim, const RegressionTreeWrapperParams& rtree_params);

    virtual ~RegressionTreeWrapper();

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::clear()
     */
    // --------------------------------------------------------------
    virtual void clear();

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::saveToFile()
     */
    // --------------------------------------------------------------
    virtual int saveToFile(const string& basename);

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::loadFromFile()
     */
    // --------------------------------------------------------------
    virtual int loadFromFile(const string& basename);

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::addTrainingSample()
     */
    // --------------------------------------------------------------
    virtual int addTrainingSample(const float* const feature_vals,
                                  const unsigned int length,
                                  const unsigned int start_idx,
                                  const float target);

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::train()
     */
    // --------------------------------------------------------------
    virtual int train();

    // --------------------------------------------------------------
    /**
     * \brief See RegressorWrapper::predict()
     */
    // --------------------------------------------------------------
    virtual int predict(const float* const feature_vals,
                        const unsigned int length,
                        const unsigned int start_idx,
                        float& predicted_val);

  private:
    RegressionTreeWrapperParams rtree_params_;

    CvDTree* rtree_;

    // Intermediate containers used by addTrainingSample
    vector<const float*> interm_feature_vals_;
    vector<unsigned int> interm_start_idx_;
    vector<unsigned int> interm_lengths_;
    vector<float> interm_target_;
};

#endif

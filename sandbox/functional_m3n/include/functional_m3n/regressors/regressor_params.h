#ifndef __REGRESSOR_PARAMS_H__
#define __REGRESSOR_PARAMS_H__
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

// --------------------------------------------------------------
//* RegressionTreeWrapperParams
/**
 * \brief Parameters to construct a RegressTreeWrapper
 */
// --------------------------------------------------------------
class RegressionTreeWrapperParams
{
  public:
    // --------------------------------------------------------------
    /**
     * \brief Default parameters for regression tree.
     *
     * See CvDTreeParams in OpenCV: \n
     * http://opencv.willowgarage.com/wiki/MachineLearning#DecisionTrees \n
     *
     * max_tree_depth_factor \n
     * min_sample_count \n
     * regression_accuracy \n
     * nbr_xvalidation_folds \n
     *
     * max_allocation The max size of the matrix to train regressor TODO
     */
    // --------------------------------------------------------------
    RegressionTreeWrapperParams() :
      max_tree_depth_factor(0.75), min_sample_count(10), regression_accuracy(0.001),
          nbr_xvalidation_folds(10), max_allocation(2e8)
    {
    }

    double max_tree_depth_factor;
    unsigned int min_sample_count;
    double regression_accuracy;
    unsigned int nbr_xvalidation_folds;
    unsigned int max_allocation;
};

#endif

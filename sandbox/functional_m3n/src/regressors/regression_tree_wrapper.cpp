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

#include <functional_m3n/regressors/regression_tree_wrapper.h>

using namespace std;

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RegressionTreeWrapper::RegressionTreeWrapper(unsigned int stacked_feature_dim)
{
  // These should never be cleared
  algorithm_type_ = REGRESSION_TREE;
  stacked_feature_dim_ = stacked_feature_dim;
  // rtree_params_ = (default constructor)

  trained_ = false;
  rtree_ = NULL;
  //interm_length_ = 0;
}

// --------------------------------------------------------------
/*!
 * See function definition
 * Invariant: the parameter values are valid (see M3NParams::setRegressorRegressionTrees)
 * */
// --------------------------------------------------------------
RegressionTreeWrapper::RegressionTreeWrapper(unsigned int stacked_feature_dim,
                                             const RegressionTreeWrapperParams& rtree_params)
{
  // These should never be cleared
  algorithm_type_ = REGRESSION_TREE;
  stacked_feature_dim_ = stacked_feature_dim;
  rtree_params_ = rtree_params;

  trained_ = false;
  rtree_ = NULL;
  //interm_length_ = 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RegressionTreeWrapper::~RegressionTreeWrapper()
{
  clear();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void RegressionTreeWrapper::clear()
{
  if (rtree_ != NULL)
  {
    delete rtree_;
  }
  rtree_ = NULL;

  trained_ = false;

  interm_feature_vals_.clear();
  interm_start_idx_.clear();
  //interm_length_ = 0;
  interm_lengths_.clear();
  interm_target_.clear();

  // do NOT clear stacked_feature_dim_, algorithm_type_, or rtree_params_
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RegressionTreeWrapper::saveToFile(const string& basename)
{
  // -------------------------------------------
  // Verify regression tree is trainedtrained
  if (!trained_)
  {
    ROS_ERROR("Cannot save untrained regression tree");
    return -1;
  }

  // -------------------------------------------
  // Create filename: <basename>_wrapper.rtree
  string out_filename = basename;
  out_filename.append("_wrapper.rtree");
  ofstream outfile(out_filename.c_str());
  if (outfile.is_open() == false)
  {
    ROS_ERROR("Could not open file %s to save to", basename.c_str());
    return -1;
  }

  // -------------------------------------------
  // Create filename: <basename>_opencv.rtree
  // Save regressor tree structure to file
  string opencv_filename = basename;
  opencv_filename.append("_opencv.rtree");
  rtree_->save(opencv_filename.c_str());

  // -------------------------------------------
  // File format:
  // algorithm_type_
  // stacked_feature_dim_
  //
  // rtree_params_
  //
  // <opencv filename string length>
  // <opencv filename>

  // -------------------------------------------
  outfile << algorithm_type_ << endl;
  outfile << stacked_feature_dim_ << endl;
  outfile << endl;
  outfile << rtree_params_.max_tree_depth_factor << endl;
  outfile << rtree_params_.min_sample_count << endl;
  outfile << rtree_params_.regression_accuracy << endl;
  outfile << rtree_params_.nbr_xvalidation_folds << endl;
  outfile << endl;
  outfile << opencv_filename.length() << endl;
  outfile << opencv_filename << endl;

  outfile.close();
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RegressionTreeWrapper::loadFromFile(const string& basename)
{
  clear();

  // -------------------------------------------
  // Create filename: <basename>_wrapper.rtree
  string in_filename = basename;
  in_filename.append("_wrapper.rtree");
  ifstream infile(in_filename.c_str());
  if (infile.is_open() == false)
  {
    ROS_ERROR("Could not open file %s to load from", basename.c_str());
    return -1;
  }

  // -------------------------------------------
  // File format:
  // algorithm_type_
  // stacked_feature_dim_
  //
  // rtree_params_
  //
  // <opencv filename string length>
  // <opencv filename>

  // -------------------------------------------
  // Verify loading a regression tree with expected stacked feature dimension
  int tempo_algo_type;
  unsigned int tempo_stacked_feature_dim;
  infile >> tempo_algo_type;
  infile >> tempo_stacked_feature_dim;
  if (tempo_algo_type != REGRESSION_TREE)
  {
    ROS_ERROR("Regressor is not a regression tree");
    return -1;
  }
  if (tempo_stacked_feature_dim != stacked_feature_dim_)
  {
    ROS_ERROR("Incompatiable stacked feature dimension");
    return -1;
  }

  infile >> rtree_params_.max_tree_depth_factor;
  infile >> rtree_params_.min_sample_count;
  infile >> rtree_params_.regression_accuracy;
  infile >> rtree_params_.nbr_xvalidation_folds;

  unsigned int opencv_filename_strlength = 0;
  infile >> opencv_filename_strlength;
  char opencv_filename[opencv_filename_strlength];
  infile >> opencv_filename;

  rtree_ = new CvDTree;
  rtree_->load(opencv_filename);
  infile.close();
  trained_ = true;
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RegressionTreeWrapper::addTrainingSample(const float* const feature_vals,
                                             const unsigned int length,
                                             const unsigned int start_idx,
                                             const float target)
{
  // -------------------------------------------
  // Verify not already trained
  if (trained_)
  {
    ROS_ERROR("Cannot add training sample because already trained");
    return -1;
  }

  // -------------------------------------------
  // Verify the feature dimension is within bounds
  if (start_idx + length > stacked_feature_dim_)
  {
    ROS_ERROR("Training sample location (%u,%u) exceeds total feature dimension %u", start_idx, length,
        stacked_feature_dim_);
    return -1;
  }

  // -------------------------------------------
  // Verify the number of features is consistent
  /*
  if (interm_length_ == 0)
  {
    interm_length_ = length;
  }
  else if (interm_length_ != length)
  {
    ROS_ERROR("length is not consistent: %u, %u", interm_length_, length);
    return -1;
  }
  */

  // -------------------------------------------
  // Save value for training stage
  interm_feature_vals_.push_back(feature_vals);
  interm_lengths_.push_back(length);
  interm_start_idx_.push_back(start_idx);
  interm_target_.push_back(target);
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RegressionTreeWrapper::train()
{
  // -------------------------------------------
  // Verify not already trained
  if (trained_)
  {
    ROS_ERROR("Cannot re-train");
    return -1;
  }

  // -------------------------------------------
  // Check number of training samples is sensible
  unsigned int nbr_samples = interm_feature_vals_.size();
  if (nbr_samples < rtree_params_.min_sample_count)
  {
    ROS_WARN("Have less training samples %u than needed to perform split %u", nbr_samples,
        rtree_params_.min_sample_count);
  }

  // -------------------------------------------
  // Vector with target values to regress to
  CvMat* target_vals = cvCreateMat(nbr_samples, 1, CV_32F);

  // -------------------------------------------
  // Create nbr_samples-by-stacked_feature_dim_ matrix to hold feature values in each row
  float* sparse_feature_matrix = static_cast<float*> (calloc((nbr_samples * stacked_feature_dim_),
      sizeof(float)));

  // -------------------------------------------
  // Populate target_vals and sparse_feature_matrix for each sample
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    // Define the target value for current sample
    cvmSet(target_vals, i, 0, interm_target_[i]);

    // Place the feature values at location sparse_feature_matrix[i][interm_start_idx_[i]
    //memcpy((sparse_feature_matrix + (i * stacked_feature_dim_) + interm_start_idx_[i]),
    //    (interm_feature_vals_[i]), (interm_length_ * sizeof(float)));
    memcpy((sparse_feature_matrix + (i * stacked_feature_dim_) + interm_start_idx_[i]),
        (interm_feature_vals_[i]), (interm_lengths_[i] * sizeof(float)));
  }

  // -------------------------------------------
  // Create OpenCV data structures to train regression tree
  CvMat train_data;
  cvInitMatHeader(&train_data, nbr_samples, stacked_feature_dim_, CV_32F, sparse_feature_matrix);
  CvMat* var_type = cvCreateMat(stacked_feature_dim_ + 1, 1, CV_8U); // TODO is this correct??
  cvSet(var_type, cvScalarAll(CV_VAR_ORDERED)); // indicates using numbers and not categories

  rtree_ = new CvDTree;
  bool train_success = rtree_->train(&train_data, // training data
      CV_ROW_SAMPLE, // how to read train_data
      target_vals, // target values
      NULL, // var_idx
      NULL, // sample_idx
      var_type, //var_type
      NULL, // missing mask
      CvDTreeParams(static_cast<int> (rtree_params_.max_tree_depth_factor * stacked_feature_dim_), // max depth
          rtree_params_.min_sample_count, // min sample count
          rtree_params_.regression_accuracy, // regression accuracy
          false, // do NOT compute surrogate split (no missing data)
          15, // TODO verify not used max number of categories (use sub-optimal algorithm for larger numbers)
          rtree_params_.nbr_xvalidation_folds, // the number of cross-validation folds
          true, // use 1SE rule => smaller tree
          true, // throw away the pruned tree branches
          NULL // priors
      ));

  free(sparse_feature_matrix);
  interm_feature_vals_.clear();
  interm_start_idx_.clear();
  interm_lengths_.clear();
  //interm_length_ = 0;
  interm_target_.clear();

  if (train_success)
  {
    trained_ = true;
    return 0;
  }
  else
  {
    return -1;
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int RegressionTreeWrapper::predict(const float* const feature_vals,
                                   const unsigned int length,
                                   const unsigned int start_idx,
                                   float& predicted_val)
{
  // TODO remove check
  if (!trained_ || (start_idx + length > stacked_feature_dim_))
  {
    ROS_ERROR("Not initialized, or exceeded feature dimensions");
    return -1;
  }

  // Create feature vec
  float* big_feature_vec = static_cast<float*> (calloc(stacked_feature_dim_, sizeof(float)));
  memcpy((big_feature_vec + start_idx), feature_vals, (length) * sizeof(float));

  // Wrap with OpenCV data structure
  CvMat cv_feature_vec;
  cvInitMatHeader(&cv_feature_vec, 1, stacked_feature_dim_, CV_32F, big_feature_vec);

  // Predict with regression tree
  predicted_val = rtree_->predict(&cv_feature_vec, NULL)->value;

  // Cleanup
  free(big_feature_vec);
  return 0;
}

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/*
 * LevMarqSparseBundleAdj.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQBUNDLEADJ_H_
#define LEVMARQBUNDLEADJ_H_

#include "LevMarqTransformDispSpace.h"
#include "PointTracks.h"

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

namespace cv { namespace willow {

/// \brief A special Levenberg-Marquardt for bundle adjustment in visual odometry.
///
/// The implementation of this class was based on the following paper,
/// "Bundle Adjustment Rules", by Chris Engles, Henrik Stewenius, and David Nister,
/// Photogrammetric Computer Vision (PCV), September 2006.
///
class LevMarqSparseBundleAdj: public LevMarqTransformDispSpace {
public:
  typedef LevMarqTransformDispSpace Parent;
  LevMarqSparseBundleAdj(
      /// transformation matrix from disparity space to Cartesian space.
      const CvMat *disparityTo3D,
      /// transformation matrix from Cartesian space to disparity space.
      const CvMat *threeDToDisparity,
      /// size of the sliding window in full size.
      int full_free_window_size,
      /// max number of fixed frames(cameras)
      int full_fixed_window_size,
      /// termination criteria. max_iter specifies the max number of iterations
      /// allowed, including both good updates and retractions in Levenberg-Marquardt
      /// epsilon specifies the maximum change in parameter to declare convergence.
      /// Specifically, we declare convergence if the relative L2-norm of
      /// the difference vector between the parameters of two consecutive iterations
      /// is less than epsilon.
      CvTermCriteria term_criteria0 =
        cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON) );
  virtual ~LevMarqSparseBundleAdj();
  // see .cpp file for detailed documentation.
  bool optimize(
      /// The set of free frames. The transformation matrix
      /// is used as initial value in entry and output in exit.
      vector<FramePose*>* freeFrames,
      /// The set of fixed frames.
      vector<FramePose*>* fixedFrames,
      /// The tracks of points. The global coordinates for each track are
      /// used as initial value in entry and output in exit.
      PointTracks* tracks
  );

  /// The cost of current iteration in optimize(). Or final cost when
  /// optimize() returns.
  double cost_;
  /// Initial cost.
  double initial_cost_;
  /// number of retraction (when error/cost has increased).
  int num_retractions_;
  /// number of good update (when error/cost has decreased).
  int num_good_updates_;

protected:
  void initParams(
      vector<FramePose*>* free_frames,
      vector<FramePose*>* fixed_frames,
      PointTracks* tracks
  );
  /// compute the cost function, for example, the 2-norm of error vector.
  inline double costFunction(
      /// The window of frames. For a free frame, the transformation matrix
      /// is used as initial value in entry and output in exit.
      vector<FramePose*>* free_frames,
      /// The tracks of points. The global coordinates for each track are
      /// used as initial value in entry and output in exit.
      PointTracks* tracks
  );
  inline void constructTransfMatrices();
  inline void constructFwdTransfMatrices();
  inline void constructFwdTransfMatrices(
      const CvMat *param, double delta, double *transf_fwds_data);
  inline void constructTransfMatrix(const CvMat* param, double _T[]);

  /// compute the relative 2-norm of the difference vector of parameters between last
  /// iteration and this one.
  inline double getParamChange(const PointTracks* tracks) const;
  /// compute the change in point parameters between last iteration and this one.
  inline void getPointParamChange(
      const PointTracks* tracks,
      double *param_diff_sum_sq,
      double *param_sum_sq) const;
  void retrieveOptimizedParams(
      vector<FramePose*>* free_frames,
      PointTracks* tracks);
  inline bool isFreeFrame(int global_frame_index) {
    if (indexmap_free_global_to_local_.find(global_frame_index) ==
      indexmap_free_global_to_local_.end() ) {
      // not in the set
      return false;
    } else {
      return true;
    }
  }
  /// returns true if it is a fixed camera
  inline bool isFixedFrame(int global_frame_index) {
    if (indexmap_fixed_global_to_local_.find(global_frame_index) ==
      indexmap_fixed_global_to_local_.end() ) {
      // not in the set
      return false;
    } else {
      return true;
    }
  }
  /// returns true if neither free camera nor fixed camera. This camera
  /// is not used in optimization.
  inline bool isDontCareFrame(int global_frame_index) {
    if (isFixedFrame(global_frame_index) == false &&
        isFreeFrame(global_frame_index)  == false) {
      return true;
    } else {
      return false;
    }
  }

  static const int NUM_POINT_PARAMS = 3;
  static const int NUM_CAM_PARAMS = numParams;
  static const int DIM = 3; // 2 for monocullar, 3 for stereo

  /// max number of free cameras/frames. At the beginning, we may not
  /// have a full slide window.
  const int full_free_window_size_;
  /// current free window size. less or equals to full_free_window_size
  int free_window_size_;

  const int full_fixed_window_size_;
  /// current fixed window size. less or equals to full_fixed_window_size.
  int fixed_window_size_;

  int num_tracks_;
  /// a data buffer that is large enough for matrix A in case of a full sliding window.
  double* A_data_;
  /// row step of matrix A. Keep in fixed even though the window may not full.
  const int A_step_;
  inline double* getABlock(int c0, int c1) {
    return &A_data_[c0*NUM_CAM_PARAMS*A_step_ + c1*NUM_CAM_PARAMS];
  }
  inline void getABlock(CvMat* mat, int c0, int c1) {
    cvGetSubRect(&mat_A_, mat,
        cvRect(c1*NUM_CAM_PARAMS, c0*NUM_CAM_PARAMS,
            NUM_CAM_PARAMS, NUM_CAM_PARAMS));
  }
  /// CvMat header for A_data_, for full sliding windows.
  CvMat mat_A_full_;
  /// CvMat header for A_data_, for current sliding window (may not be full)
  CvMat mat_A_;
  /// a data buffer that is large enough for matrix B in case of a full sliding window.
  double* B_data_;
  inline double* getBBlock(int c) {
    return &B_data_[c*NUM_CAM_PARAMS];
  }
  /// CvMat header for B_data_, for full sliding windows.
  CvMat mat_B_full_;
  /// CvMat header for B_data_, for the current sliding window (may not be full)
  CvMat mat_B_;

  boost::unordered_map<int, CvMat *> map_global_to_disp_;
//  boost::unordered_map<int, int> map_index_global_to_local_;
  boost::unordered_map<int, int> indexmap_free_global_to_local_;
  boost::unordered_map<int, int> indexmap_fixed_global_to_local_;
  /// camera(frame) parameters. 6 each.
  double* frame_params_;
  /// get the pointer to the parameters of frame i.
  inline double* getFrameParams(int i) { return &frame_params_[i*6];}
  /// CvMat header for frame_params_. Initialized in method optimize()
  CvMat mat_C_;
  /// camera(frame) parameters from last step. 6 for each camera.
  double* frame_prev_params_;
  /// get the pointer to the previous parameters of frame i.
  inline double* getFramePrevParams(int i) { return &frame_prev_params_[i*6];}
  /// CvMat header for frame_prev_params_. Initialized in method optimize()
  CvMat mat_prev_C_;
  /// update vector of the frame parameters at current iteration
  double* frame_params_update_;
  /// get the pointer to the parameter updates of frame i
  inline double* getFrameParamsUpdate(int i) {
    return &frame_params_update_[i*6];
  }
  /// CvMat header for previous parameters. Initialized in method optimize()
  CvMat mat_dC_;

  /// a data buffer that is large enough for all transformation matrices
  /// of the free frames from global
  /// to local disparity space, 4x4 for each,
  /// up to full_fixed_window_size*16.
  double* free_transf_data_;
  /// a data buffer that is large enough for all transformation matrices
  /// of the free frames from global
  /// to local disparity space, 4x4 for each,
  /// up to full_fixed_window_size*16.
  double* fixed_transf_data_;
  inline double* getFreeTransf(int i) {
    return &free_transf_data_[i*16];
  }
  inline double* getFixedTransf(int i){
    return &fixed_transf_data_[i*16];
  }
  inline double* getTransf(int global_index, int local_index) {
    if (isFreeFrame(global_index) == true) {
      return getFreeTransf(local_index);
    } else if (isDontCareFrame(global_index) == false ) {
      return getFixedTransf(local_index);
    } else { // it is an old frame
      return NULL;
    }
  }
  /// a data buffer that is large enough for transformation matrices from global
  /// to local disparity space, of forwarded
  /// parameters of each frame(camera). 6x(4x4) for each camera.
  double * transf_fwd_data_;
  inline double * getTransfFwd(int iFrame, int iParam) {
    return &(transf_fwd_data_[iFrame*6*16 + iParam*16]);
  }
  /// Compute the Jacobian of the point parameters numerically,
  /// by forward finite difference.
  inline void JacobianOfPointNumeric(
      double px, double py, double pz,
      double pu, double pv, double pd,
      double rx, double ry, double rz,
      /// inverse of delta
      double scale,
      double *transf_global_disp,
      double *Jp
  );
  /// Compute the Jacobian of the point parameters analytically.
  inline void JacobianOfPointAnalytic(
      PointTrackObserv* obsv,
      double *transf_global_disp,
      double *Jp
  );
  /// \brief Solving the linear system with Cholesky factorization.
  /// Cholesky factor the left hand side matrix A and
  /// solve for dC.
  /// (Alternatively, we may use a special SVD for symmetric square matrix
  /// in OpenCV, which is slower than the Cholesky in eigen2)
  void linearSolving();

  /// Levenberg-Marquardt scalar. Initialized to zero
  double lambdaLg10_;
  CvTermCriteria term_criteria_;
  /// @todo move this constant upward to its parents
  /// parameter delta value. Used for compute Jacobian numerically.
  const double param_delta_;

  /// cost of previous iteration.
  double prev_cost_;

};

}

}

#endif /* LEVMARQBUNDLEADJ_H_ */

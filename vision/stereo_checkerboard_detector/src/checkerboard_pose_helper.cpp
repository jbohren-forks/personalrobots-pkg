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

//! \author Vijay Pradeep

#include "stereo_checkerboard_detector/checkerboard_pose_helper.h"

#include "tf/transform_datatypes.h"


using namespace stereo_checkerboard_detector ;

void CheckerboardPoseHelper::setSize(int w, int h, double spacing)
{
  if (cb_expected_)
    cvReleaseMat(&cb_expected_) ;

  cb_expected_ = cvCreateMat(w*h, 3, CV_32FC1) ;

  // Builds the [correctly ordered] list of points in the checkerboard
  for (int j=0; j<h; j++)
  {
    for(int i=0; i<w; i++)
    {
      int ind = i + j*w ;
      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_, ind, 0)) = i*spacing ;
      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_, ind, 1)) = j*spacing ;
      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_, ind, 2)) = 0.0 ;
    }
  }
}

double CheckerboardPoseHelper::getPose(const CvMat* cb_sensed, tf::Pose& pose_tf)
{
  float R[9] ;
  float trans[3] ;
  CvMat cv_R = cvMat(3, 3, CV_32FC1, R) ;
  CvMat cv_trans = cvMat(3, 1, CV_32FC1, trans) ;

  double rms_err ;
  rms_err = getPose(cb_sensed, &cv_R, &cv_trans) ;
  btMatrix3x3 rot3x3(R[0], R[1], R[2],
                     R[3], R[4], R[5],
                     R[6], R[7], R[8]);

  printf("R =\n") ;
  for (int i=0; i<3; i++)
  {
    printf("  % .6f, % .6f, % .6f\n", *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 0)),
                                      *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 1)),
                                      *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 2)) ) ;
  }

  tf::Quaternion q ;
  rot3x3.getRotation(q) ;

  printf("Length=%f\n", q.length()) ;
  printf("q_raw = [%f, %f, %f, %f]\n", q.x(), q.y(), q.z(), q.w()) ;
  q.normalize() ;
  printf("q_norm = [%f, %f, %f, %f]\n", q.x(), q.y(), q.z(), q.w()) ;

  pose_tf = tf::Pose(q, tf::Vector3(trans[0], trans[1], trans[2])) ;

  printf("Trans = (%f,%f,%f)\n", trans[0], trans[1], trans[2]) ;

  return rms_err ;
}

double CheckerboardPoseHelper::getPose(const CvMat* cb_sensed, CvMat* R, CvMat* trans)
{
  int N = cb_sensed->height ;

  if (cvGetSize(cb_sensed).width != 3)
    ROS_ERROR("Array Width must be 3") ;

  if (N != cvGetSize(cb_expected_).height)
  {
    ROS_ERROR("Array heights don't match. cb_expected.height=%i cb_sensed.height=%i",
              cb_expected_->height, cb_sensed->height ) ;
  }

  printf("cb_sensed =\n") ;
  //  for (int i=0; i<N; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *cb_sensed, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_sensed, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_sensed, i, 2)) ) ;
  //  }


  // Build a matrix full of ones
  CvMat* ones_vec = cvCreateMat(N,1, CV_32FC1) ;
  //printf("ones_mat.rows()=%i\n", ones_mat->rows) ;
  //printf("ones_mat.cols()=%i\n", ones_mat->cols) ;
  for (int i=0; i<N; i++)
  {
    *( (float*) CV_MAT_ELEM_PTR( *ones_vec, i, 0)) = 1.0 ;
    //*( (float*) CV_MAT_ELEM_PTR( *ones_mat, i, 0)) = 1.0 ;
    //*( (float*) CV_MAT_ELEM_PTR( *ones_mat, i, 1)) = 1.0 ;
    //*( (float*) CV_MAT_ELEM_PTR( *ones_mat, i, 2)) = 1.0 ;
  }

  float avg_sensed[3] = {0.0, 0.0, 0.0} ;
  float avg_expected[3] = {0.0, 0.0, 0.0} ;
  CvMat cb_avg_sensed   = cvMat(3,1,CV_32FC1, avg_sensed) ;
  CvMat cb_avg_expected = cvMat(3,1,CV_32FC1, avg_expected) ;


  // Avg = A'*[1] / N
  cvGEMM(cb_sensed, ones_vec, 1.0/N, NULL, 0.0, &cb_avg_sensed, CV_GEMM_A_T) ;
  cvGEMM(cb_expected_, ones_vec, 1.0/N, NULL, 0.0, &cb_avg_expected, CV_GEMM_A_T) ;

  printf("avg_sensed=%f, %f, %f\n", avg_sensed[0], avg_sensed[1], avg_sensed[2]) ;
  printf("avg_expected=%f, %f, %f\n", avg_expected[0], avg_expected[1], avg_expected[2]) ;

  CvMat* cb_sensed_centered   = cvCreateMat(N, 3, CV_32FC1) ;
  CvMat* cb_expected_centered = cvCreateMat(N, 3, CV_32FC1) ;

  // centered = orig - [1 ... 1]'*[avgx avgy avgz]
  cvGEMM(ones_vec, &cb_avg_sensed,   -1.0, cb_sensed,    1.0, cb_sensed_centered,   CV_GEMM_B_T) ;
  cvGEMM(ones_vec, &cb_avg_expected, -1.0, cb_expected_, 1.0, cb_expected_centered, CV_GEMM_B_T) ;

  //  printf("centered_expected:\n") ;
  //  for (int i=0; i<N; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 2)) ) ;
  //  }

  CvMat* outer_prod = cvCreateMat(3,3, CV_32FC1) ;
  // outer_prod = sensed'*expected
  cvGEMM(cb_sensed_centered, cb_expected_centered, 1.0, NULL, 0.0, outer_prod, CV_GEMM_A_T) ;

  //  printf("cv_sensed_centered =\n") ;
  //  for (int i=0; i<N; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *cb_sensed_centered, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_sensed_centered, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_sensed_centered, i, 2)) ) ;
  //  }

  //  printf("cv_expected_centered =\n") ;
  //  for (int i=0; i<N; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *cb_expected_centered, i, 2)) ) ;
  //  }


  //  printf("outer_prod =\n") ;
  //  for (int i=0; i<3; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *outer_prod, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *outer_prod, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( *outer_prod, i, 2)) ) ;
  //  }


  CvMat* U = cvCreateMat(3,3, CV_32FC1) ;
  CvMat* S = cvCreateMat(3,3, CV_32FC1) ;
  CvMat* V = cvCreateMat(3,3, CV_32FC1) ;

  cvSVD(outer_prod, S, U, V) ;

  printf("U = [\n") ;
  for (int i=0; i<3; i++)
  {
    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *U, i, 0)),
                                      *( (float*) CV_MAT_ELEM_PTR( *U, i, 1)),
                                      *( (float*) CV_MAT_ELEM_PTR( *U, i, 2)) ) ;
  }
  printf("];\n") ;

  printf("S =[\n") ;
  for (int i=0; i<3; i++)
  {
    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *S, i, 0)),
                                      *( (float*) CV_MAT_ELEM_PTR( *S, i, 1)),
                                      *( (float*) CV_MAT_ELEM_PTR( *S, i, 2)) ) ;
  }
  printf("];\n") ;

  printf("V =[\n") ;
  for (int i=0; i<3; i++)
  {
    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( *V, i, 0)),
                                      *( (float*) CV_MAT_ELEM_PTR( *V, i, 1)),
                                      *( (float*) CV_MAT_ELEM_PTR( *V, i, 2)) ) ;
  }
  printf("];\n") ;

  double detU = det3x3(U) ;
  double detV = det3x3(V) ;
  printf("det(U)=%f\n", detU) ;
  printf("det(V)=%f\n", detV) ;
  if (detU*detV < 0)
  {
    // Negate the last row of V
    for (int i=0; i<3; i++)
      *( (float*) CV_MAT_ELEM_PTR( *V, 2, i)) = -*( (float*) CV_MAT_ELEM_PTR( *V, 2, i)) ;
  }

  // R = U*V'
  cvGEMM(U, V, 1.0, NULL, 0.0, R, CV_GEMM_B_T) ;

  // T = sensed_avg - R*expected_avg
  cvGEMM(R, &cb_avg_expected, -1.0, &cb_avg_sensed, 1.0, trans) ;

  cvReleaseMat(&V) ;
  cvReleaseMat(&S) ;
  cvReleaseMat(&U) ;
  cvReleaseMat(&outer_prod) ;
  cvReleaseMat(&cb_expected_centered) ;
  cvReleaseMat(&cb_sensed_centered) ;
  cvReleaseMat(&ones_vec) ;

  return -1 ;   //! \todo populate this with the RMS distance error for the fitted pose
}

double CheckerboardPoseHelper::det3x3(const CvMat* A)
{
  return (*((float*)CV_MAT_ELEM_PTR( *A, 0, 0)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 1)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 2)))
       + (*((float*)CV_MAT_ELEM_PTR( *A, 0, 1)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 2)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 0)))
       + (*((float*)CV_MAT_ELEM_PTR( *A, 0, 2)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 0)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 1)))
       - (*((float*)CV_MAT_ELEM_PTR( *A, 0, 0)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 2)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 1)))
       - (*((float*)CV_MAT_ELEM_PTR( *A, 0, 1)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 0)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 2)))
       - (*((float*)CV_MAT_ELEM_PTR( *A, 0, 2)))*(*((float*)CV_MAT_ELEM_PTR( *A, 1, 1)))*(*((float*)CV_MAT_ELEM_PTR( *A, 2, 0))) ;
}

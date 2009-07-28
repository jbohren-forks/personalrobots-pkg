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

#include "stereo_checkerboard_detector/reprojection_helper.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;


void ReprojectionHelper::computeDisparity(const vector<geometry_msgs::Point>& left_pts,
                                          const vector<geometry_msgs::Point>& right_pts,
                                          std::vector<geometry_msgs::Point>& result)
{
  ROS_ERROR("Not Yet Implemented") ;
}



void ReprojectionHelper::computeDisparity(const vector<CvPoint2D32f>& left_pts,
                                          const vector<CvPoint2D32f>& right_pts,
                                          CvMat* uvd)
{
  if (!uvd)
  {
    ROS_ERROR("Must allocate output array ahead of time") ;
    return ;
  }

  if (left_pts.size() != right_pts.size())
  {
    ROS_ERROR("left and right list lengths don't match") ;
    return ;
  }

  if (cvGetSize(uvd).width != 1)
  {
    ROS_ERROR("Output Array must have width 1. (cvGetSize(xyd).width = %i)", cvGetSize(uvd).width) ;
    return ;
  }

  if (cvGetSize(uvd).height != (int)left_pts.size())
  {
    ROS_ERROR("Output Array height must match left_pts.size(). (cvGetSize(xyd).height = %i)", cvGetSize(uvd).height) ;
    return ;
  }

  for (unsigned int i=0; i<left_pts.size(); i++)
  {
    float d = left_pts[i].x - right_pts[i].x ;
    // Copy into output array
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->u = left_pts[i].x ;
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->v = left_pts[i].y ;
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->d = d ;
  }

}

void ReprojectionHelper::reproject(const std::vector<geometry_msgs::Point>& ros_uvd,
                                   const sensor_msgs::CamInfo& left_info,
                                   const sensor_msgs::CamInfo& right_info,
                                   std::vector<geometry_msgs::Point>& ros_xyz)
{
  const unsigned int N = ros_uvd.size() ;

  CvMat* uvd = cvCreateMat(N, 1, CV_32FC3) ;
  CvMat* xyz = cvCreateMat(N, 1, CV_32FC3) ;

  for (unsigned int i=0; i<N; i++)
  {
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->u = ros_uvd[i].x ;
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->v = ros_uvd[i].y ;
    ((Disp_Point_t*) CV_MAT_ELEM_PTR(*uvd, i, 0))->d = ros_uvd[i].z ;
  }

  reproject(uvd, left_info, right_info, xyz) ;

  ros_xyz.resize(N) ;
  for (unsigned int i=0; i<N; i++)
  {
    ros_xyz[i].x = ((Cart_Point_t*) CV_MAT_ELEM_PTR(*xyz, i, 0))->x ;
    ros_xyz[i].y = ((Cart_Point_t*) CV_MAT_ELEM_PTR(*xyz, i, 0))->y ;
    ros_xyz[i].z = ((Cart_Point_t*) CV_MAT_ELEM_PTR(*xyz, i, 0))->z ;
  }

  cvReleaseMat(&uvd) ;
  cvReleaseMat(&xyz) ;
}

void ReprojectionHelper::reproject(const CvMat* uvd,
                                   const sensor_msgs::CamInfo& left_info,
                                   const sensor_msgs::CamInfo& right_info,
                                   CvMat* xyz)
{
  unsigned int N = uvd->rows ;

  // Build reprojection matrix
  CvMat* Q = cvCreateMat(4, 4, CV_32FC1) ;
  buildQ(left_info, right_info, Q) ;

  //  for (int i=0; i<4; i++)
  //  {
  //    for(int j=0; j<4; j++)
  //    {
  //      printf(" %.2f  ", * (float*) CV_MAT_ELEM_PTR(*Q,i,j)) ;
  //    }
  //    printf("\n") ;
  //  }

  //cvPerspectiveTransform(uvd, xyz, Q) ;

  //! \todo Use cvPerspectiveTransform to do this!
  //****** Hack code *******
  float* mat = Q->data.fl ;
  for( unsigned int i = 0; i < N; i++ )
  {
    const float* src = (float*) ( (char*)uvd->data.fl + uvd->step*i);
    float* dst = (float*) ( (char*)xyz->data.fl + xyz->step*i);

    float x = src[0], y = src[1], z = src[2];


    float w = x*mat[12] + y*mat[13] + z*mat[14] + mat[15];
    if( fabs(w) > FLT_EPSILON )
    {
      w = 1./w;
      dst[0]   = ((x*mat[0] + y*mat[1] + z*mat[2] + mat[3]) * w);
      dst[1] = ((x*mat[4] + y*mat[5] + z*mat[6] + mat[7]) * w);
      dst[2] = ((x*mat[8] + y*mat[9] + z*mat[10] + mat[11]) * w);
    }
    else
      dst[0] = dst[1] = dst[2] = 0;
  }
  //************************

  cvReleaseMat(&Q) ;

}

void ReprojectionHelper::buildQ(const sensor_msgs::CamInfo& left_info,
                                const sensor_msgs::CamInfo& right_info,
                                CvMat* Q)
{
  // Clear Q
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
      *(float*)CV_MAT_ELEM_PTR(*Q, i, j) = 0.0 ;

  // Extract pertinent human readable elem names
  double cx   =  left_info.P[2] ;
  double cx_r = right_info.P[2] ;
  double cy   =  left_info.P[6] ;
  double Fx   =  left_info.P[0] ;
  double Fy   =  left_info.P[5] ;
  double Tx   = -right_info.P[3] / left_info.P[0] ;

  * (float*) CV_MAT_ELEM_PTR(*Q,0,0) =  1.0  ;
  * (float*) CV_MAT_ELEM_PTR(*Q,0,3) = -cx ;
  * (float*) CV_MAT_ELEM_PTR(*Q,1,1) =  Fx / Fy ;
  * (float*) CV_MAT_ELEM_PTR(*Q,1,3) = -cy * Fx / Fy ;
  * (float*) CV_MAT_ELEM_PTR(*Q,2,3) =  Fx ;
  * (float*) CV_MAT_ELEM_PTR(*Q,3,2) =  1/(Tx) ;
  * (float*) CV_MAT_ELEM_PTR(*Q,3,3) = -(cx-cx_r)/(Tx) ;

  // Load up reprojection matrix Q
  //  [1  0    0  -Cx
  //   0  1    0  -Cy
  //   0  0    0   f
  //   0  0  -1/Tx (Cx-Cx')/Tx ]
//  * (float*) CV_MAT_ELEM_PTR(*Q,0,0) =  1.0 ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,0,3) = -cx ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,1,1) =  1.0 ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,1,3) = -cy ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,2,3) =  Fx ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,3,2) =  1/(Tx) ;
//  * (float*) CV_MAT_ELEM_PTR(*Q,3,3) = -(cx-cx_r)/(Tx) ;


}



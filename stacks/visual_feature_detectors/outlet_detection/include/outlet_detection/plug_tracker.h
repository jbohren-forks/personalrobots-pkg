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

#ifndef PLUG_TRACKER_H
#define PLUG_TRACKER_H

#include "outlet_detection/tracker_base.h"

class PlugTracker : public TrackerBase
{
public:
  PlugTracker(ros::Node &node);
  ~PlugTracker();

protected:
  virtual bool detectObject(tf::Transform &pose);
  virtual CvRect getBoundingBox();
  virtual IplImage* getDisplayImage(bool success);
  void getBoardCorners(CvPoint2D32f corners[4]);

  void publishBoardMarker(const tf::Transform &board_in_cam);
  void publishBoardRayMarker(const tf::Transform &board_in_cam);
  void publishPlugRayMarker(const tf::Transform &board_in_cam,
                            const tf::Transform &plug_pose);

  int board_w_, board_h_;
  CvMat *grid_pts_;
  int ncorners_;
  std::vector<CvPoint2D32f> corners_;

  tf::Transform plug_in_board_, camera_in_cvcam_;
  tf::Transform prong_in_board_;
  
  cv::WImageBuffer3_b display_img_;
};

#endif

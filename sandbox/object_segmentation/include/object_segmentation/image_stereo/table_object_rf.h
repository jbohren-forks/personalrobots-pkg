#ifndef __TABLE_OBJECT_RF_H__
#define __TABLE_OBJECT_RF_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.hpp"

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/cloud_io.h>

#include <point_cloud_clustering/all_clusterings.h>

#include <descriptors_3d/all_descriptors.h>

#include <functional_m3n/m3n_model.h>
#include <functional_m3n/random_field.h>

#include <object_segmentation/util/rf_creator_3d.h>

class TableObjectRF
{
  public:
    static const std::string CHANNEL_ARRAY_WIDTH;
    static const std::string CHANNEL_ARRAY_HEIGHT;
    static const std::string CHANNEL_LABEL;

    TableObjectRF();

    boost::shared_ptr<RandomField> createRandomField(const string& fname_image,
                                                     const string& fname_pcd);

  private:
    int loadStereoImageCloud(const string& fname_image,
                             const string& fname_pcd,
                             IplImage* image,
                             sensor_msgs::PointCloud* stereo_cloud);

    void
    downsampleStereoCloud(sensor_msgs::PointCloud& full_stereo_cloud,
                          sensor_msgs::PointCloud& ds_stereo_cloud,
                          const double voxel_x,
                          const double voxel_y,
                          const double voxel_z,
                          vector<unsigned int>& ds_labels,
                          map<unsigned int, pair<unsigned int, unsigned int> >& ds_idx2img_coords);

    RFCreator3D* rf_creator_3d;
};

#endif

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

#include <stdlib.h>

#include <vector>
#include <iostream>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <descriptors_3d/bounding_box.h>
#include <descriptors_3d/orientation.h>
#include <descriptors_3d/position.h>
#include <descriptors_3d/spectral_shape.h>
#include <descriptors_3d/spin_image.h>

using namespace std;

void createPointCloud(robot_msgs::PointCloud& data);

// --------------------------------------------------------------
/*!
 * \brief Example program demonstrating how to use the desciptors_3d package.
 */
// --------------------------------------------------------------
int main()
{
  SpectralShape spectral_shape;
  spectral_shape.setSpectralRadius(5.0);

  // Compute a spin image centered at each interest point.
  // The spin axis here is the +z direction
  // Note that the number of rows in the spin image MUST be odd
  // (see documentation)
  SpinImage spin_image1;
  spin_image1.setAxisCustom(0.0, 0.0, 1.0);
  spin_image1.setImageDimensions(1.0, 1.0, 5, 4);

  // Compute another spin image centered at each interest point.
  // The spin axis here is about each interest points' estimated normal.
  // We can either recompute the normals using a different radius than
  // used with spectral_shape, or we can use the same normals computed
  // from spectral_shape.
  SpinImage spin_image2;
  spin_image2.setAxisNormal();
  //spin_image2.setSpectralRadius(0.2286); // this will recompute normals!
  spin_image2.useSpectralInformation(&spectral_shape);
  spin_image2.setImageDimensions(1.0, 1.0, 5, 4);

  // Compares the locally estimated normal and tangent vectors around
  // each interest point against the specified reference direction (+z).
  // The feature value is cos(theta), where theta is the angle between
  // the normal/tangent and the reference direction
  Orientation orientation;
  orientation.useNormalOrientation(0.0, 0.0, 1.0);
  orientation.useTangentOrientation(0.0, 0.0, 1.0);
  orientation.useSpectralInformation(&spectral_shape);

  // The feature is simply the z coordinate for each interest point
  Position position;

  // Computes the bounding box
  BoundingBox bounding_box(true, false);
  bounding_box.useSpectralInformation(&spectral_shape);
  bounding_box.setBoundingBoxRadius(5.0);

  // ----------------------------------------------
  //
  vector<Descriptor3D*> descriptors_3d;
  descriptors_3d.push_back(&spectral_shape);
  descriptors_3d.push_back(&spin_image1);
  descriptors_3d.push_back(&spin_image2);
  descriptors_3d.push_back(&orientation);
  descriptors_3d.push_back(&position);
  descriptors_3d.push_back(&bounding_box);

  // ----------------------------------------------
  // Read point cloud data and create Kd-tree that represents points.
  // We will compute features for all points in the point cloud.
  robot_msgs::PointCloud data;
  createPointCloud(data);
  cloud_kdtree::KdTreeANN data_kdtree(data);
  cv::Vector<const robot_msgs::Point32*> interest_pts(data.pts.size());
  for (unsigned int i = 0 ; i < data.pts.size() ; i++)
  {
    interest_pts[i] = &(data.pts[i]);
  }

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud.
  // The compute() populates a vector of vector of floats, i.e. a feature vector for each
  // interest point.  If the features couldn't be computed successfully for an interest point,
  // its feature vector has size 0
  unsigned int nbr_descriptors = descriptors_3d.size();
  vector<cv::Vector<cv::Vector<float> > > all_descriptor_results(nbr_descriptors);
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    descriptors_3d[i]->compute(data, data_kdtree, interest_pts, all_descriptor_results[i]);
  }

  // Print out the bounding box dimension features for the first point 0
  cv::Vector<float>& pt0_bbox_features = all_descriptor_results[5][0];
  cout << "Bounding box features:";
  for (size_t i = 0 ; i < pt0_bbox_features.size() ; i++)
  {
    cout << " " << pt0_bbox_features[i];
  }
  cout << endl;
}

// Generates random point cloud
void createPointCloud(robot_msgs::PointCloud& data)
{
  unsigned int nbr_pts = 5000;
  data.pts.resize(nbr_pts);

  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    data.pts[i].x = rand() % 50;
    data.pts[i].y = rand() % 50;
    data.pts[i].z = rand() % 50;
  }
}


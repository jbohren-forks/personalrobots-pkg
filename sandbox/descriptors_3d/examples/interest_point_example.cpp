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

#include <vector>

#include <descriptors_3d/bounding_box.h>
#include <descriptors_3d/orientation.h>
#include <descriptors_3d/position.h>
#include <descriptors_3d/spectral_shape.h>
#include <descriptors_3d/spin_image.h>

using namespace std;

int main()
{
  SpectralShape spectral_shape;
  spectral_shape.setSpectralRadius(0.2286);

  // Compute a spin image centered at each interest point.
  // The spin axis here is the +z direction
  // Note that the number of rows in the spin image MUST be odd
  // (see documentation)
  SpinImage spin_image1;
  spin_image1.setAxisCustom(0.0, 0.0, 1.0);
  spin_image1.setImageDimensions(0.0762, 0.0762, 5, 4);

  // Compute another spin image centered at each interest point.
  // The spin axis here is about each interest points' estimated normal.
  // We can either recompute the normals using a different radius than
  // used with spectral_shape, or we can use the same normals computed
  // from spectral_shape.
  SpinImage spin_image2;
  spin_image2.setAxisNormal();
  //spin_image2.setSpectralRadius(0.2286); // this will recompute normals!
  spin_image2.useSpectralInformation(&spectral_shape);
  spin_image2.setImageDimensions(0.0762, 0.0762, 5, 4);

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
  // Iterate over each descriptor and compute features for each point in the point cloud.
  // The compute() populates a vector of vector of floats, i.e. a feature vector for each
  // interest point.  If the features couldn't be computed successfully for an interest point,
  // its feature vector has size 0
  unsigned int nbr_descriptors = descriptors_3d.size();
  vector<cv::Vector<cv::Vector<float> > > all_descriptor_results(nbr_descriptors);
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    //descriptors_3d[i]->compute(data, data_kdtree, interest_pts, all_descriptor_results[i]);
  }

}

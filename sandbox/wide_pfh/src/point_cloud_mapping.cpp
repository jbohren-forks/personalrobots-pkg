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

#include "wide_pfh/point_cloud_mapping.h"


// functions that should eventually go into the point_cloud_mapping package

namespace cloud_geometry {

namespace statistics
{

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief GB Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
* in a given pointcloud, without considering points outside of a distance threshold from the laser origin
* \param xyzd            pointer to a 2D float CvMat of xyzd points where "d" is disparity and 0=> invalid.
* \param indices         integer offsets into xyzd indicating the points to use.  No invalid points allowed here
* \param min_pt         the resultant minimum bounds
* \param max_pt         the resultant maximum bounds
* \param cut_distance a maximum admissible distance threshold (z distance) for points from the imager origin
*/
inline void
getMinMax (const CvMat *xyzd, const std::vector<int> &indices,
		geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
		double cut_distance)
{
	min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
	max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;
	float *px = xyzd->data.fl, *py = xyzd->data.fl+1, *pz = xyzd->data.fl+2, *pd = xyzd->data.fl+3; //Set up our base pointers
	for (unsigned int i = 0; i < indices.size (); i++)
	{
		if((*(pz+indices[i]) > cut_distance) || (*(pd+indices[i]) == 0.0)) continue; //Do not allow invalid, or too far away points
		min_pt.x = (*(px + indices[i]) < min_pt.x) ? *(px +indices[i]) : min_pt.x;
		min_pt.y = (*(py + indices[i]) < min_pt.y) ? *(py +indices[i]) : min_pt.y;
		min_pt.z = (*(pz + indices[i]) < min_pt.z) ? *(pz +indices[i]) : min_pt.z;

		max_pt.x = (*(px + indices[i]) > max_pt.x) ? *(px +indices[i]) : max_pt.x;
		max_pt.y = (*(py + indices[i]) > max_pt.y) ? *(py +indices[i]) : max_pt.y;
		max_pt.z = (*(pz + indices[i]) > max_pt.z) ? *(pz +indices[i]) : max_pt.z;
	}
}


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief GB Downsample a Point Cloud using a voxelized grid approach
  * \param xyzd            pointer to a 2D float CvMat of xyzd points where "d" is disparity and 0=> invalid.
  * \param indices         integer offsets into xyzd indicating the points to use.  No invalid points allowed here
  * \param points_down     integer offsets into xyzd indicating the resultant downsampled points
  * \param leaf_size         the voxel leaf dimensions
  * \param leaves         a vector of already existing leaves (empty for the first call)
  * \param cut_distance     the maximum admissible distance of a point from the viewpoint (default: DBL_MAX)
  */
void
  downsamplePointCloud (const CvMat *xyzd, const std::vector<int> &indices, std::vector<int> &points_down,
		  geometry_msgs::Point leaf_size, std::vector<Leaf> &leaves, double cut_distance)
{

  // allocate enough space for points
  points_down.resize(indices.size());

  geometry_msgs::Point32 min_p, max_p, min_b, max_b, div_b;
  cloud_geometry::statistics::getMinMax (xyzd, indices, min_p, max_p, cut_distance);
  float *px = xyzd->data.fl, *py = xyzd->data.fl+1, *pz = xyzd->data.fl+2, *pd = xyzd->data.fl+3; //Set up our base pointers

  // Compute the minimum and maximum bounding box values
  min_b.x = (int)(floor (min_p.x / leaf_size.x));
  max_b.x = (int)(floor (max_p.x / leaf_size.x));

  min_b.y = (int)(floor (min_p.y / leaf_size.y));
  max_b.y = (int)(floor (max_p.y / leaf_size.y));

  min_b.z = (int)(floor (min_p.z / leaf_size.z));
  max_b.z = (int)(floor (max_p.z / leaf_size.z));

  // Compute the number of divisions needed along all axis
  div_b.x = (int)(max_b.x - min_b.x + 1);
  div_b.y = (int)(max_b.y - min_b.y + 1);
  div_b.z = (int)(max_b.z - min_b.z + 1);

  // Allocate the space needed
  try
  {
    if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
      leaves.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
    leaves.resize (div_b.x * div_b.y * div_b.z);
  }
  catch (std::bad_alloc)
  {
    ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
               div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
  }
  std::vector<std::vector<int> > leaf_pts(leaves.size()); //Record points for each leaf

  for (unsigned int cl = 0; cl < leaves.size (); cl++)
  {
    if (leaves[cl].nr_points > 0)
    {
      leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
      leaves[cl].nr_points = 0;
    }
  }

  // First pass: go over all points and insert them into the right leaf
  for (unsigned int cp = 0; cp < indices.size (); cp++)
  {
      if((*(pz+indices[cp]) > cut_distance) || (*(pd+indices[cp]) == 0.0)) continue; //Do not allow invalid, or too far away points

    int i = (int)(floor (*(px + indices[cp]) / leaf_size.x));
    int j = (int)(floor (*(py + indices[cp]) / leaf_size.y));
    int k = (int)(floor (*(pz + indices[cp]) / leaf_size.z));

    int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
    leaves[idx].centroid_x += *(px + indices[cp]);
    leaves[idx].centroid_y += *(py + indices[cp]);
    leaves[idx].centroid_z += *(pz + indices[cp]);
    leaf_pts[idx].push_back(indices[cp]);         //Record the offset to this point
    leaves[idx].nr_points++;
  }

  // Second pass: go over all leaves and compute centroids
  int nr_p = 0,offset,min_offset;
  float Xc,Yc,Zc,dX,dY,dZ,dist2,min_dist2;
  for (unsigned int cl = 0; cl < leaves.size (); cl++)
  {
    if (leaves[cl].nr_points > 0)
    {
      Xc = leaves[cl].centroid_x / leaves[cl].nr_points;
      Yc = leaves[cl].centroid_y / leaves[cl].nr_points;
      Zc = leaves[cl].centroid_z / leaves[cl].nr_points;
      min_dist2 = 10000000.0;
      min_offset = 0;
      for(unsigned int i = 0; i<leaf_pts[cl].size(); ++i){ //Go through the points for this leaf and find the one closest to the centroid
          offset = leaf_pts[cl][i];
          dX = *(px + offset) - Xc;
          dY = *(py + offset) - Yc;
          dZ = *(pz + offset) - Zc;
          dist2 = dX*dX + dY*dY + dZ*dZ;
          if(dist2 < min_dist2){
              min_dist2 = dist2;
              min_offset = offset;
          }
      }
      points_down[nr_p] = min_offset;
      nr_p++;
    }
  }
  points_down.resize (nr_p);
}
}

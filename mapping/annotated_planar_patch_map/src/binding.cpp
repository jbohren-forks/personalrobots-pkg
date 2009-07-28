/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*
* Author: Alexander Sorokin
*********************************************************************/

#include "point_cloud_mapping/geometry/areas.h"


#include "annotated_planar_patch_map/annotated_map_lib.h"
#include "point_cloud_mapping/kdtree/kdtree_ann.h"

using namespace annotated_map_lib;



std::vector<int> computeCorrespondence(const annotated_map_msgs::TaggedPolygonalMap& map_from, const annotated_map_msgs::TaggedPolygonalMap& map_to, double max_radius)
{

  std::vector<int> result;

  //Compute the center of mass for each planar polygon
  unsigned int num_poly_from=map_from.polygons.size();
  if(num_poly_from==0)
  {
    result.resize(0);
    return result;
  }


  sensor_msgs::PointCloud centers_from;
  centers_from.pts.resize(num_poly_from);
  result.resize(num_poly_from);
  
  for(unsigned int iPoly=0;iPoly<num_poly_from;iPoly++){
    centers_from.pts[iPoly]=annotated_map_lib::computeMean(map_from.polygons[iPoly].polygon);
    result[iPoly]=-1;
  }

  unsigned int num_poly_to=map_to.polygons.size();
  sensor_msgs::PointCloud centers_to;
  centers_to.pts.resize(num_poly_to);
  
  for(unsigned int iPoly=0;iPoly<num_poly_to;iPoly++){
    centers_to.pts[iPoly]=annotated_map_lib::computeMean(map_to.polygons[iPoly].polygon);
  }

  ROS_DEBUG("Building KDtree");
  //Build kd-tree on centers of mass
  cloud_kdtree::KdTreeANN kd_tree(centers_to);

  
  for(unsigned int iPoly=0;iPoly<num_poly_from;iPoly++){
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    kd_tree.radiusSearch(centers_to, iPoly, max_radius, k_indices, k_distances);
    if(k_indices.size()==0)
      continue;
    if(k_distances[0]>max_radius)
      continue;

    result[iPoly]=k_indices[0];
  }

}

void transferAnnotations(const annotated_map_msgs::TaggedPolygonalMap& map_with_annotations, const annotated_map_msgs::TaggedPolygonalMap& map_to_bind, annotated_map_msgs::TaggedPolygonalMap& map_out,double max_radius)
{
  //it has the size of map_to_bind. -1 if no correpondence;
  std::vector<int> correspondence=computeCorrespondence(map_to_bind,map_with_annotations,max_radius );

  bool bSame=false;
  if(&map_out==&map_to_bind)
    bSame=true;

  int num_poly_out = 0;
  for(int iPoly=0;iPoly<int(correspondence.size());iPoly++)
  {
    int jPoly=correspondence[iPoly];
    if(jPoly==-1)
      continue;
    num_poly_out++;
  }
  map_out.polygons.resize(num_poly_out);
  int i_poly_out=0;
  for(int iPoly=0;iPoly<int(correspondence.size());iPoly++)
  {
    int jPoly=correspondence[iPoly];
    if(jPoly==-1)
      continue;


    map_out.polygons[i_poly_out] = map_to_bind.polygons[iPoly];
    copyPolygonTags(map_with_annotations.polygons[jPoly],
                         map_out.polygons[i_poly_out]);
    i_poly_out++;
    
  }
  
  map_out.header=map_to_bind.header;
}


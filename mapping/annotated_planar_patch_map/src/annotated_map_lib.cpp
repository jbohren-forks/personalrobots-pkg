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
using namespace annotated_map_lib;



void annotated_map_lib::transformAnyObject(const std::string & target_frame, 
                                           const tf::Transform* net_transform, 
                                           const mapping_msgs::PolygonalMap & polymapIn, 
                                           mapping_msgs::PolygonalMap & polymapOut)
{
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(*net_transform);
  
  bool bSame = (&polymapIn == &polymapOut);
  unsigned int num_polygons = polymapIn.get_polygons_size();
  if (!bSame)
  {
    polymapOut.set_polygons_size(polymapIn.get_polygons_size());
  }
  
  for(unsigned int iPoly=0;iPoly<num_polygons;iPoly++)  
  {    
    const robot_msgs::Polygon3D* p=&polymapIn.polygons[iPoly];
    
    unsigned int length = p->get_points_size();
    
    boost::numeric::ublas::matrix<double> matIn(4, length);
    
    //  double * matrixPtr = matIn.Store();
    
    for (unsigned int i = 0; i < length ; i++)
    {
      matIn(0,i) = p->points[i].x;
      matIn(1,i) = p->points[i].y;
      matIn(2,i) = p->points[i].z;
      matIn(3,i) = 1;
    };
    
    boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);
    
    robot_msgs::Polygon3D *polyOut;
    if (!bSame)
    {
      polyOut = &(polymapOut.polygons[iPoly]);
      polyOut->set_points_size(length);
      polyOut->color=p->color;
    }
    else
    {
      polyOut = &(polymapOut.polygons[iPoly]);
    }
    for (unsigned int i = 0; i < length ; i++)
    {
      polyOut->points[i].x = matOut(0,i);
      polyOut->points[i].y = matOut(1,i);
      polyOut->points[i].z = matOut(2,i);
    };
    
  }
  // Copy relevant data from polymapIn, if needed
  if (! bSame )
  {
    polymapOut.header = polymapIn.header;
    polymapOut.set_chan_size(polymapIn.get_chan_size());
    for (unsigned int i = 0 ; i < polymapIn.get_chan_size() ; ++i)
      polymapOut.chan[i] = polymapIn.chan[i];
  }
  
  //Override the positions
  polymapOut.header.frame_id = target_frame;
  
}



void annotated_map_lib::transformAnyObject(const std::string & target_frame, 
                        const tf::Transform* net_transform, 
                        const annotated_map_msgs::TaggedPolygonalMap & polymapIn, 
                        annotated_map_msgs::TaggedPolygonalMap & polymapOut)
{
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(*net_transform);
  
  bool bSame = (&polymapIn == &polymapOut);
  unsigned int num_polygons = polymapIn.get_polygons_size();
  if (!bSame)
  {
    polymapOut.set_polygons_size(polymapIn.get_polygons_size());
  }
  
  for(unsigned int iPoly=0;iPoly<num_polygons;iPoly++)  
  {    
    const annotated_map_msgs::TaggedPolygon3D* p=&polymapIn.polygons[iPoly];
    
    unsigned int length = p->polygon.get_points_size();
    
    boost::numeric::ublas::matrix<double> matIn(4, length);
    
    //  double * matrixPtr = matIn.Store();
    
    for (unsigned int i = 0; i < length ; i++)
    {
      matIn(0,i) = p->polygon.points[i].x;
      matIn(1,i) = p->polygon.points[i].y;
      matIn(2,i) = p->polygon.points[i].z;
      matIn(3,i) = 1;
    };
    
    boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);
    
    annotated_map_msgs::TaggedPolygon3D *polyOut;
    if (!bSame)
    {
      polyOut = &(polymapOut.polygons[iPoly]);
      polyOut->polygon.set_points_size(length);
      polyOut->polygon.color=p->polygon.color;
      polyOut->tags=p->tags;
      polyOut->tags_chan=p->tags_chan;
    }
    else
    {
      polyOut = &(polymapOut.polygons[iPoly]);
    }
    for (unsigned int i = 0; i < length ; i++)
    {
      polyOut->polygon.points[i].x = matOut(0,i);
      polyOut->polygon.points[i].y = matOut(1,i);
      polyOut->polygon.points[i].z = matOut(2,i);
    };
    
  }
  // Copy relevant data from polymapIn, if needed
  if (! bSame )
  {
    polymapOut.header = polymapIn.header;
    /*polymapOut.set_chan_size(polymapIn.get_chan_size());
      for (unsigned int i = 0 ; i < polymapIn.get_chan_size() ; ++i)
      polymapOut.chan[i] = polymapIn.chan[i];
    */
  }
  
  //Override the positions
  polymapOut.header.frame_id = target_frame;
  

}

void annotated_map_lib::copyPolygonTags(const annotated_map_msgs::TaggedPolygon3D &polyIn,annotated_map_msgs::TaggedPolygon3D &polyOut)
{
  polyOut.tags=polyIn.tags;
  polyOut.tags_chan=polyIn.tags_chan;
}

void annotated_map_lib::transformAnyObject(const std::string & target_frame, 
                        const tf::Transform* net_transform, 
                        const annotated_map_msgs::TaggedPolygon3D & polyIn, 
                        annotated_map_msgs::TaggedPolygon3D & polyOut)
{
  ROS_BREAK();
}




/** \brief Convert the transform to a Homogeneous matrix for large operations */
boost::numeric::ublas::matrix<double> annotated_map_lib::transformAsMatrix(const tf::Transform& bt)
{
  boost::numeric::ublas::matrix<double> outMat(4,4);
  
  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);
  
  tf::Vector3 origin = bt.getOrigin();
  
  outMat(0,0)= mv[0];
  outMat(0,1)  = mv[4];
  outMat(0,2)  = mv[8];
  outMat(1,0)  = mv[1];
  outMat(1,1)  = mv[5];
  outMat(1,2)  = mv[9];
  outMat(2,0)  = mv[2];
  outMat(2,1)  = mv[6];
  outMat(2,2) = mv[10];
  
  outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
  outMat(0,3) = origin.x();
  outMat(1,3) = origin.y();
  outMat(2,3) = origin.z();
  outMat(3,3) = 1;  
  
  return outMat;
};

 

boost::unordered_map<std::string, int> annotated_map_lib::getAllMapTags(const annotated_map_msgs::TaggedPolygonalMap& map){
  
  boost::unordered_map<std::string, int> output_dict;
  int num_polygons=map.polygons.size();
  for(int iPoly=0;iPoly<num_polygons;iPoly++)
  {
    int num_tags=map.polygons[iPoly].tags.size();
    for(int iT=0;iT<num_tags;iT++)
    {
      std::string tag_name=map.polygons[iPoly].tags[iT];
      if ( output_dict.find(tag_name)==output_dict.end())
        output_dict[tag_name]=1;
      else
        output_dict[tag_name]+=1;
    }
  }

  return output_dict;

}




double annotated_map_lib::getMapArea(const annotated_map_msgs::TaggedPolygonalMap& map)
{
  double tot_area=0;
  unsigned int num_poly=map.polygons.size();
  for(unsigned int iPoly=0;iPoly<num_poly;iPoly++)
  {
    tot_area+=cloud_geometry::areas::compute2DPolygonalArea(map.polygons[iPoly].polygon);
  }
  return tot_area;
}

/* !
 * 
 * \brief Get the area of all polygons that have all the tags in the query 
 *
 *
 */
double annotated_map_lib::getMapAreaWithTagsMatchAll(const annotated_map_msgs::TaggedPolygonalMap& map,std::vector<std::string> query_tags)
{
  double tot_area=0;
  unsigned int num_poly=map.polygons.size();
  for(unsigned int iPoly=0;iPoly<num_poly;iPoly++)
  {
    if(annotated_map_lib::doesQueryMatchAll(query_tags,map.polygons[iPoly]))
       tot_area+=cloud_geometry::areas::compute2DPolygonalArea(map.polygons[iPoly].polygon);
  }
  return tot_area;
}

/* !
 * 
 * \brief Get the area of all polygons that have any of tags in the query 
 *
 *
 */
double annotated_map_lib::getMapAreaWithTagsMatchAny(const annotated_map_msgs::TaggedPolygonalMap& map,std::vector<std::string> query_tags)
{
  double tot_area=0;
  unsigned int num_poly=map.polygons.size();
  for(unsigned int iPoly=0;iPoly<num_poly;iPoly++)
  {
    if(annotated_map_lib::doesQueryMatchAny(query_tags,map.polygons[iPoly]))
       tot_area+=cloud_geometry::areas::compute2DPolygonalArea(map.polygons[iPoly].polygon);
  }
  return tot_area;
}


 
geometry_msgs::Point32 annotated_map_lib::computeMean (const robot_msgs::Polygon3D& poly)
{
  geometry_msgs::Point32 mean;
  mean.x=0;mean.y=0;mean.z=0;
  
  unsigned int sz= poly.points.size();;
  for (unsigned int i = 0; i < sz; i++)
  {
    mean.x += poly.points[i].x;
    mean.y += poly.points[i].y;
    mean.z += poly.points[i].z;
  }
  if(sz>0){
	mean.x /= sz;
	mean.y /= sz;
	mean.z /= sz;
  }
  return mean;
}


